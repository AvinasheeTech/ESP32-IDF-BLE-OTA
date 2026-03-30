package com.avinasheetech.bleota

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.provider.OpenableColumns
import android.util.Log
import android.widget.*
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import java.util.*
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

class MainActivity : AppCompatActivity() {

    // ─── UUIDs (must match ESP32 exactly) ───────────────────────────────────
    companion object {
        private const val TAG = "BLE_OTA"

        private val OTA_SERVICE_UUID = UUID.fromString("6f9742f3-97b2-b594-f343-74a44f52d0d2")
        private val OTA_CONTROL_UUID = UUID.fromString("0000ee01-0000-1000-8000-00805f9b34fb")
        private val OTA_DATA_UUID    = UUID.fromString("0000ee02-0000-1000-8000-00805f9b34fb")
        private val CCCD_UUID        = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        // OTA flags
        private val OTA_REQUEST     = byteArrayOf(0x01)
        private val OTA_REQUEST_ACK = byteArrayOf(0x02)
        private val OTA_DONE        = byteArrayOf(0x04)
        private val OTA_DONE_ACK    = byteArrayOf(0x05)

        private const val MANUF_ID          = 0x02E5
        private const val PACKET_SIZE       = 253
        private const val PERM_REQUEST_CODE = 100
    }

    // ─── UI ──────────────────────────────────────────────────────────────────
    private lateinit var tvDeviceStatus: TextView
    private lateinit var tvFileName:     TextView
    private lateinit var tvProgress:     TextView
    private lateinit var tvPacketInfo:   TextView
    private lateinit var tvLog:          TextView
    private lateinit var progressBar:    ProgressBar
    private lateinit var btnScan:        Button
    private lateinit var btnSelectFile:  Button
    private lateinit var btnStartOta:    Button
    private lateinit var scrollLog:      ScrollView

    // ─── BLE state ───────────────────────────────────────────────────────────
    private var bluetoothGatt:   BluetoothGatt?      = null
    private var bleScanner:      BluetoothLeScanner? = null
    private var connectedDevice: BluetoothDevice?    = null
    private var firmwareUri:     Uri?                = null
    private var isScanning       = false
    private var otaRunning       = false

    // Latch for notification responses (OTA_REQUEST_ACK / OTA_DONE_ACK)
    private var responseLatch:    CountDownLatch? = null
    private var lastNotification: ByteArray?      = null

    // THE KEY FIX: one latch per write — blocks until onCharacteristicWrite fires.
    // Without this, Android's BLE stack queue overflows (~33% through) and silently
    // drops writes while the loop keeps incrementing progress. This serialises every
    // single write so we never send the next packet until the stack confirms the previous.
    private var writeLatch: CountDownLatch? = null

    private val mainHandler = Handler(Looper.getMainLooper())
    private val scanHandler = Handler(Looper.getMainLooper())

    // ─── File picker ─────────────────────────────────────────────────────────
    private val filePicker = registerForActivityResult(
        ActivityResultContracts.GetContent()
    ) { uri: Uri? ->
        uri?.let {
            firmwareUri = it
            tvFileName.text = getFileName(it)
            log("Selected firmware: ${tvFileName.text}")
            updateStartButton()
        }
    }

    // ─── Lifecycle ───────────────────────────────────────────────────────────
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        tvDeviceStatus = findViewById(R.id.tvDeviceStatus)
        tvFileName     = findViewById(R.id.tvFileName)
        tvProgress     = findViewById(R.id.tvProgress)
        tvPacketInfo   = findViewById(R.id.tvPacketInfo)
        tvLog          = findViewById(R.id.tvLog)
        progressBar    = findViewById(R.id.progressBar)
        btnScan        = findViewById(R.id.btnScan)
        btnSelectFile  = findViewById(R.id.btnSelectFile)
        btnStartOta    = findViewById(R.id.btnStartOta)
        scrollLog      = findViewById(R.id.scrollLog)

        btnScan.setOnClickListener       { checkPermissionsAndScan() }
        btnSelectFile.setOnClickListener { filePicker.launch("application/octet-stream") }
        btnStartOta.setOnClickListener   { startOtaInBackground() }

        checkPermissionsAndScan()
    }

    override fun onDestroy() {
        super.onDestroy()
        disconnectGatt()
    }

    // ─── Permissions ─────────────────────────────────────────────────────────
    private fun requiredPermissions() =
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S)
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        else
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)

    private fun hasPermissions() = requiredPermissions().all {
        ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
    }

    private fun checkPermissionsAndScan() {
        if (!hasPermissions())
            ActivityCompat.requestPermissions(this, requiredPermissions(), PERM_REQUEST_CODE)
        else
            startScan()
    }

    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<out String>, grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == PERM_REQUEST_CODE &&
            grantResults.all { it == PackageManager.PERMISSION_GRANTED })
            startScan()
        else
            log("ERROR: Permissions denied.")
    }

    // ─── Scan ────────────────────────────────────────────────────────────────
    @SuppressLint("MissingPermission")
    private fun startScan() {
        if (isScanning) return
        val adapter = (getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
        if (adapter == null || !adapter.isEnabled) { log("ERROR: Bluetooth not enabled."); return }

        disconnectGatt()
        connectedDevice     = null
        tvDeviceStatus.text = "Scanning..."
        btnScan.isEnabled   = false
        log("Scanning for ESP32 OTA device...")

        bleScanner = adapter.bluetoothLeScanner
        isScanning = true
        bleScanner?.startScan(scanCallback)

        scanHandler.postDelayed({
            stopScan()
            if (connectedDevice == null) {
                log("Scan complete. Device not found.")
                tvDeviceStatus.text = "Device not found"
                btnScan.isEnabled   = true
            }
        }, 10_000)
    }

    @SuppressLint("MissingPermission")
    private fun stopScan() {
        if (isScanning) { bleScanner?.stopScan(scanCallback); isScanning = false }
    }

    @SuppressLint("MissingPermission")
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device     = result.device
            val scanRecord = result.scanRecord ?: return
            val manufData  = scanRecord.manufacturerSpecificData
            if (manufData != null && manufData.indexOfKey(MANUF_ID) >= 0) {
                val hasOtaService = scanRecord.serviceUuids?.any { it.uuid == OTA_SERVICE_UUID } == true
                if (hasOtaService) {
                    stopScan()
                    log("Device found: ${device.address}")
                    tvDeviceStatus.text = "Found: ${device.address}\nConnecting..."
                    connectToDevice(device)
                }
            }
        }
        override fun onScanFailed(errorCode: Int) {
            log("Scan failed: $errorCode"); isScanning = false; btnScan.isEnabled = true
        }
    }

    // ─── Connect ─────────────────────────────────────────────────────────────
    @SuppressLint("MissingPermission")
    private fun connectToDevice(device: BluetoothDevice) {
        bluetoothGatt = device.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
    }

    @SuppressLint("MissingPermission")
    private fun disconnectGatt() {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
    }

    // ─── GATT Callback ───────────────────────────────────────────────────────
    @SuppressLint("MissingPermission")
    private val gattCallback = object : BluetoothGattCallback() {

        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    log("Connected. Discovering services...")
                    connectedDevice = gatt.device
                    gatt.discoverServices()
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    log("Disconnected.")
                    mainHandler.post {
                        tvDeviceStatus.text = "Disconnected"
                        btnScan.isEnabled   = true
                        updateStartButton()
                    }
                    // Release any hanging latches so the OTA thread exits cleanly
                    writeLatch?.countDown()
                    responseLatch?.countDown()
                }
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                if (gatt.getService(OTA_SERVICE_UUID) != null) {
                    log("OTA service found. Requesting MTU 256...")
                    gatt.requestMtu(256)
                } else {
                    log("ERROR: OTA service not found on device.")
                    mainHandler.post { tvDeviceStatus.text = "OTA service missing" }
                }
            } else {
                log("Service discovery failed: $status")
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS)
                log("MTU negotiated: $mtu (payload: ${mtu - 3} bytes). Ready.")
            else
                log("MTU request failed ($status), using default.")
            mainHandler.post {
                tvDeviceStatus.text = "Connected: ${gatt.device.address}"
                btnScan.isEnabled   = true
                updateStartButton()
            }
        }

        // *** Releases writeLatch so the OTA thread can send the next packet ***
        override fun onCharacteristicWrite(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int
        ) {
            if (status != BluetoothGatt.GATT_SUCCESS)
                log("Write failed on ${characteristic.uuid}, status: $status")
            writeLatch?.countDown()
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            if (characteristic.uuid == OTA_CONTROL_UUID) {
                lastNotification = characteristic.value.copyOf()
                log("Notification: ${characteristic.value.toHexString()}")
                responseLatch?.countDown()
            }
        }

        // Descriptor writes also go through the BLE queue — release writeLatch here too
        override fun onDescriptorWrite(
            gatt: BluetoothGatt,
            descriptor: BluetoothGattDescriptor,
            status: Int
        ) {
            if (status == BluetoothGatt.GATT_SUCCESS)
                log("Descriptor write OK.")
            writeLatch?.countDown()
        }
    }

    // ─── OTA ─────────────────────────────────────────────────────────────────
    private fun startOtaInBackground() {
        if (otaRunning) return
        otaRunning              = true
        btnStartOta.isEnabled   = false
        btnScan.isEnabled       = false
        btnSelectFile.isEnabled = false
        resetProgress()
        Thread { performOta() }.start()
    }

    @SuppressLint("MissingPermission")
    private fun performOta() {
        try {
            val gatt     = bluetoothGatt ?: throw Exception("Not connected.")
            val uri      = firmwareUri   ?: throw Exception("No firmware file selected.")
            val service  = gatt.getService(OTA_SERVICE_UUID)           ?: throw Exception("OTA service not found.")
            val ctrlChar = service.getCharacteristic(OTA_CONTROL_UUID) ?: throw Exception("Control characteristic not found.")
            val dataChar = service.getCharacteristic(OTA_DATA_UUID)    ?: throw Exception("Data characteristic not found.")

            // Step 1: Enable notifications on Control characteristic
            log("Enabling notifications...")
            enableNotifications(gatt, ctrlChar)

            // Step 2: Send packet size (2-byte little-endian) to Data characteristic
            log("Sending packet size: $PACKET_SIZE bytes")
            writeAndWait(gatt, dataChar, byteArrayOf(
                (PACKET_SIZE and 0xFF).toByte(),
                ((PACKET_SIZE shr 8) and 0xFF).toByte()
            ))

            // Step 3: Send OTA_REQUEST
            log("Sending OTA_REQUEST (0x01)...")
            writeAndWait(gatt, ctrlChar, OTA_REQUEST)

            // Step 4: Wait for OTA_REQUEST_ACK notification
            val startAck = waitForNotification(3000)
            if (startAck == null || !startAck.contentEquals(OTA_REQUEST_ACK))
                throw Exception("OTA start failed. Got: ${startAck?.toHexString() ?: "timeout"}")
            log("OTA_REQUEST_ACK received. Starting transfer...")

            // Step 5: Send firmware chunks — each write is confirmed before the next is sent
            val chunks = readFirmwareChunks(uri)
            val total  = chunks.size
            log("Total packets: $total")

            for ((index, chunk) in chunks.withIndex()) {
                writeAndWait(gatt, dataChar, chunk)

                val packetNum = index + 1
                val progress  = (packetNum * 100) / total
                mainHandler.post {
                    progressBar.progress = progress
                    tvProgress.text      = "$progress%"
                    tvPacketInfo.text    = "Packet: $packetNum / $total"
                }
                if (packetNum % 10 == 0 || packetNum == total)
                    log("Sent $packetNum / $total")
            }

            // Step 6: Send OTA_DONE
            log("All packets sent. Sending OTA_DONE (0x04)...")
            writeAndWait(gatt, ctrlChar, OTA_DONE)

            // Step 7: Wait for OTA_DONE_ACK (long timeout — ESP32 is flashing)
            val doneAck = waitForNotification(10_000)
            if (doneAck == null || !doneAck.contentEquals(OTA_DONE_ACK))
                throw Exception("OTA done failed. Got: ${doneAck?.toHexString() ?: "timeout"}")
            log("OTA_DONE_ACK received. Firmware update successful!")

            // Step 8: Disable notifications
            disableNotifications(gatt, ctrlChar)

            mainHandler.post {
                progressBar.progress = 100
                tvProgress.text      = "100%"
                Toast.makeText(this, "OTA Update Complete!", Toast.LENGTH_LONG).show()
            }

        } catch (e: Exception) {
            log("ERROR: ${e.message}")
            Log.e(TAG, "OTA failed", e)
        } finally {
            otaRunning = false
            mainHandler.post {
                btnStartOta.isEnabled   = true
                btnScan.isEnabled       = true
                btnSelectFile.isEnabled = true
                updateStartButton()
            }
        }
    }

    // ─── BLE helpers ─────────────────────────────────────────────────────────

    // Write and block until onCharacteristicWrite confirms delivery.
    // This is what prevents the BLE queue overflow (the 33% freeze bug).
    @SuppressLint("MissingPermission")
    private fun writeAndWait(
        gatt: BluetoothGatt,
        characteristic: BluetoothGattCharacteristic,
        data: ByteArray,
        timeoutMs: Long = 2000
    ) {
        writeLatch = CountDownLatch(1)
        characteristic.value     = data
        characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        gatt.writeCharacteristic(characteristic)
        val completed = writeLatch!!.await(timeoutMs, TimeUnit.MILLISECONDS)
        writeLatch = null
        if (!completed) throw Exception("Write timed out on ${characteristic.uuid}")
    }

    @SuppressLint("MissingPermission")
    private fun enableNotifications(gatt: BluetoothGatt, char: BluetoothGattCharacteristic) {
        gatt.setCharacteristicNotification(char, true)
        val descriptor = char.getDescriptor(CCCD_UUID) ?: return
        writeLatch     = CountDownLatch(1)
        descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        gatt.writeDescriptor(descriptor)
        writeLatch!!.await(2000, TimeUnit.MILLISECONDS)
        writeLatch = null
    }

    @SuppressLint("MissingPermission")
    private fun disableNotifications(gatt: BluetoothGatt, char: BluetoothGattCharacteristic) {
        gatt.setCharacteristicNotification(char, false)
        val descriptor = char.getDescriptor(CCCD_UUID) ?: return
        descriptor.value = BluetoothGattDescriptor.DISABLE_NOTIFICATION_VALUE
        gatt.writeDescriptor(descriptor)
        Thread.sleep(200)
    }

    private fun waitForNotification(timeoutMs: Long): ByteArray? {
        lastNotification = null
        responseLatch    = CountDownLatch(1)
        val received     = responseLatch!!.await(timeoutMs, TimeUnit.MILLISECONDS)
        responseLatch    = null
        return if (received) lastNotification else null
    }

    // ─── Firmware chunking ───────────────────────────────────────────────────
    private fun readFirmwareChunks(uri: Uri): List<ByteArray> {
        val chunks = mutableListOf<ByteArray>()
        contentResolver.openInputStream(uri)?.use { stream ->
            val buffer = ByteArray(PACKET_SIZE)
            var n: Int
            while (stream.read(buffer).also { n = it } != -1)
                chunks.add(buffer.copyOf(n))
        }
        return chunks
    }

    // ─── Utilities ───────────────────────────────────────────────────────────
    private fun ByteArray.toHexString() = joinToString(" ") { "0x%02X".format(it) }

    private fun getFileName(uri: Uri): String {
        var name = "firmware.bin"
        contentResolver.query(uri, null, null, null, null)?.use { cursor ->
            val idx = cursor.getColumnIndex(OpenableColumns.DISPLAY_NAME)
            if (cursor.moveToFirst() && idx != -1) name = cursor.getString(idx)
        }
        return name
    }

    private fun updateStartButton() {
        btnStartOta.isEnabled =
            bluetoothGatt != null && connectedDevice != null && firmwareUri != null && !otaRunning
    }

    private fun resetProgress() {
        mainHandler.post {
            progressBar.progress = 0
            tvProgress.text      = "0%"
            tvPacketInfo.text    = "Packet: 0 / 0"
        }
    }

    // SCROLL FIX: use tvLog.post{} so the scroll happens after the new text
    // has been measured and laid out — not before.
//    private fun log(message: String) {
//        Log.d(TAG, message)
//        mainHandler.post {
//            val ts = java.text.SimpleDateFormat("HH:mm:ss", Locale.getDefault()).format(Date())
//            tvLog.append("[$ts] $message\n")
//            tvLog.post { scrollLog.fullScroll(ScrollView.FOCUS_DOWN) }
//        }
//    }

    private fun log(message: String) {
        Log.d(TAG, message)
        mainHandler.post {
            val ts = java.text.SimpleDateFormat("HH:mm:ss", Locale.getDefault()).format(Date())
            tvLog.append("[$ts] $message\n")
//            Log.d("SCROLL", "tvLog.height=${tvLog.height} tvLog.bottom=${tvLog.bottom} scrollLog.height=${scrollLog.height} scrollLog.scrollY=${scrollLog.scrollY} maxScroll=${tvLog.height - scrollLog.height}")
            scrollLog.post {
                scrollLog.fullScroll(ScrollView.FOCUS_DOWN)
//                Log.d("SCROLL", "after fullScroll: scrollLog.scrollY=${scrollLog.scrollY}")
            }
        }
    }
}
