import pyrealsense2 as rs
import numpy as np
import cv2

def list_realsense_devices():
    """列出所有连接的RealSense设备"""
    ctx = rs.context()
    devices = ctx.query_devices()
    device_list = []
    for i, dev in enumerate(devices):
        device_info = {}
        device_info['name'] = dev.get_info(rs.camera_info.name)
        device_info['serial'] = dev.get_info(rs.camera_info.serial_number)
        device_list.append(device_info)
        print(f"Device {i}: {device_info['name']} (Serial: {device_info['serial']})")
    return device_list

def select_realsense_device(serial_number):
    """选择并启动指定的RealSense设备"""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial_number)
    pipeline.start(config)
    return pipeline

def start_camera_pipeline(pipeline):
    """启动摄像头数据流并显示图像"""
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # 处理图像数据
            color_image = np.asanyarray(color_frame.get_data())
            cv2.imshow('RealSense', color_image)

            if cv2.waitKey(1) == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

def main():
    devices = list_realsense_devices()

    if not devices:
        print("No RealSense devices found.")
        return

    # 选择设备
    selected_serial = input("Enter the serial number of the device to open: ")

    # 检查所选的设备是否存在
    if not any(device['serial'] == selected_serial for device in devices):
        print(f"No device with serial number {selected_serial} found.")
        return

    # 启动选定的设备
    pipeline = select_realsense_device(selected_serial)
    start_camera_pipeline(pipeline)

if __name__ == "__main__":
    main()
