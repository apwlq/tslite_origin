"""
License: MIT
Author: apwlq
"""

import cv2
import time
import math
import serial
import numpy as np
import pyrealsense2 as rs

# 기본 변수 설정
moter1 = 0
moter2 = 0
handle = 0
front_light = "false"
back_light = "false"
blink = "off"
handle_relay = "false"
battery_relay = "false"

ser = serial.Serial("/dev/ttyUSB0", baudrate=9600)
time.sleep(3)

front_light = "true"

디버그모드 = False
핸들오차범위 = 0
평균 = 5
모터속도 = 100

# 이전 5개의 각도 값을 저장할 리스트
angle_history = []

def detect_steering_angle(frame):
    # 중앙에 가로로 긴 직사각형 영역(사각 박스)을 생성함
    height, width, _ = frame.shape
    roi_width = int(width * 0.7)
    roi_height = int(height * 0.2)
    roi_x = int((width - roi_width) / 2)
    roi_y = int(height * 0.4)

    # 마스크를 생성하여 사각 박스 영역만 남김
    mask = np.zeros_like(frame)
    cv2.rectangle(mask, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (255, 255, 255), thickness=cv2.FILLED)
    roi_frame = cv2.bitwise_and(frame, mask)

    # 그레이스케일 변환
    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)

    # 가우시안 블러 적용
    blurred = cv2.GaussianBlur(gray, (5,5), 1.4)

    # 캐니 엣지 검출
    canny = cv2.Canny(blurred, 50, 50)

    # 비디오 출력
    if 디버그모드 == True:
        cv2.imshow('gray', gray)
        cv2.imshow('gaussian', blurred)
        cv2.imshow('canny', canny)

    # 허프 변환을 사용하여 선을 검출
    lines = cv2.HoughLinesP(canny, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=50)

    # 선이 없으면 None 반환
    if lines is None:
        return None

    # 중앙을 기준으로 왼쪽과 오른쪽 선을 구분하기 위한 변수 초기화
    left_lines = []
    right_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        # 선의 중점 계산
        center_x = (x1 + x2) / 2

        # 중앙보다 왼쪽 혹은 오른쪽에 있는 경우
        if center_x < width / 2:
            left_lines.append(line[0])
        else:
            right_lines.append(line[0])

    # 양쪽에 선이 있는지 확인
    if not left_lines or not right_lines:
        return None

    # 왼쪽과 오른쪽 선의 중점을 계산하여 각도를 추출
    left_center = np.mean(left_lines, axis=0)
    right_center = np.mean(right_lines, axis=0)

    # 두 중점 사이의 거리를 계산하여 각도 추출
    angle = np.arctan2(right_center[1] - left_center[1], right_center[0] - left_center[0])

    # 각도를 라디안에서 도로 변환
    angle_degrees = np.degrees(angle * 4) - 핸들오차범위

    # 로봇의 한계인 45도를 넘지 않도록 제한
    if angle_degrees > 45:
        angle_degrees = 45
    elif angle_degrees < -45:
        angle_degrees = -45

    # 이전 각도를 저장하고 평균을 계산
    angle_history.append(angle_degrees)
    if len(angle_history) > 평균:
        angle_history.pop(0)
    average_angle = sum(angle_history) / len(angle_history)

    # 마스크 박스와 선을 프레임에 그립니다.
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (255, 255, 255), 1)  # 마스크 박스
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(frame, (x1, y1), (x2, y2), (100, 0, 255), 3)  # 선

    return average_angle

if __name__ == "__main__":
    # Configure the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # You can adjust the resolution and FPS

    # Start the pipeline
    pipeline.start(config)

    # Serial port setup
    ser = serial.Serial("/dev/ttyUSB0", baudrate=9600)
    time.sleep(3)

    front_light = "true"
    handle_relay = "false"

    while True:
        # Get RealSense frames
        frames = pipeline.wait_for_frames()
        frame = frames.get_color_frame()
        frame_data = np.asanyarray(frame.get_data())

        angle = detect_steering_angle(frame_data)

        if angle is not None:
            text = f"Steering Angle: {math.trunc(angle)} degrees"
            org = (50, 100)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, text, org, font, 1, (255, 0, 0), 2)

            height, width, _ = frame.shape

            # 세로 선 그리기
            line_color = (0, 255, 0)  # 초록색 (BGR 순서)
            line_thickness = 2  # 선 두께
            start_point = (math.trunc(width / 2) + math.trunc(-angle), math.trunc(height / 2) + 100)  # 선의 시작점 (x, y)
            end_point = (math.trunc(width / 2) + math.trunc(-angle), math.trunc(height / 2) - 100)  # 선의 끝점 (x, y)
            cv2.line(frame, start_point, end_point, line_color, line_thickness)
            # 가로 선 그리기
            line_color = (0, 255, 0)  # 초록색 (BGR 순서)
            line_thickness = 2  # 선 두께
            start_point = (math.trunc(width / 2), math.trunc(height / 2))  # 선의 시작점 (x, y)
            end_point = (math.trunc(width / 2) + math.trunc(-angle), math.trunc(height / 2))  # 선의 끝점 (x, y)
            cv2.line(frame, start_point, end_point, line_color, line_thickness)
            # 세로 선 그리기
            line_color = (0, 255, 255)  # 초록색 (BGR 순서)
            line_thickness = 2  # 선 두께
            start_point = (math.trunc(width / 2), math.trunc(height))  # 선의 시작점 (x, y)
            end_point = (math.trunc(width / 2), 0)  # 선의 끝점 (x, y)
            cv2.line(frame, start_point, end_point, line_color, line_thickness)

            # Update motor and handle values
            moter1 = 모터속도+angle
            moter2 = 모터속도-angle
            handle = f"{str(math.trunc(angle))}"
            command = f"#{str(moter1)} {str(moter2)} {str(handle)} {str(front_light)} {str(back_light)} {str(blink)} {str(handle_relay)} {str(battery_relay)}\n".encode('utf-8')
            ser.write(command)
            print(command)

        # Display the frame
        cv2.imshow('Steering Angle Detection', frame_data)

        # Exit the loop when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the RealSense pipeline
    pipeline.stop()

    # Close OpenCV windows
    cv2.destroyAllWindows()

# Close the serial port
moter1 = "0"
moter2 = "0"
handle = "0"
command = f"#{str(moter1)} {str(moter2)} {str(handle)} {str(front_light)} {str(back_light)} {str(blink)} {str(handle_relay)} {str(battery_relay)}\n".encode('utf-8')
ser.write(command)
print(command)
ser.close()
