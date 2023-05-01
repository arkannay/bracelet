// filename: blecontroller.dart
import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:get/get.dart';
import 'dart:async';
import 'dart:typed_data';

import 'package:permission_handler/permission_handler.dart';

class BleController {
  final frb = FlutterReactiveBle();
  late StreamSubscription<ConnectionStateUpdate> c;
  late QualifiedCharacteristic tx;
  final devId = '43:43:A2:12:1F:AC'; // use nrf connect from playstore to find
  var status = 'connect to bluetooth'.obs;
  var sliderVal = 0.0.obs;
  var currentSpeed = 0;
  var currentDirection = 0;
  List<int> packet = [0, 0];

  void sendData(val) async {
    packet[0] = val.toInt();
    await frb.writeCharacteristicWithoutResponse(tx, value: packet);
  }

  void connect() async {
    status.value = 'connecting...';
    // You can request multiple permissions at once.
    Map<Permission, PermissionStatus> statuses =
        await [Permission.bluetoothConnect, Permission.bluetoothScan].request();
    c = frb.connectToDevice(id: devId).listen((state) {
      if (state.connectionState == DeviceConnectionState.connected) {
        status.value = 'connected!';

        tx = QualifiedCharacteristic(
            serviceId: Uuid.parse("6e400001-b5a3-f393-e0a9-e50e24dcca9e"),
            characteristicId:
                Uuid.parse("6e400002-b5a3-f393-e0a9-e50e24dcca9e"),
            deviceId: devId);
      }
    });
  }
}
