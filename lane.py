import math
import cv2
import numpy as np

video_path = "video.mkv"  # 분석할 비디오 파일 경로를 여기에 입력하세요.
핸들오차범위 = -17
평균 = 25

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
    canny = cv2.Canny(blurred, 50, 150)

    # 비디오 출력
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

    # 이전 5개의 각도를 저장하고 평균을 계산
    angle_history.append(angle_degrees)
    if len(angle_history) > 평균:
        angle_history.pop(0)
    average_angle = sum(angle_history) / len(angle_history)

    # 마스크 박스와 선을 프레임에 그립니다.
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (255, 0, 0), 1)  # 마스크 박스
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 5)  # 선

    return average_angle

if __name__ == "__main__":
    cap = cv2.VideoCapture(video_path)
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        angle = detect_steering_angle(frame)

        if angle is not None:
            text = f"Steering Angle: {math.trunc(angle)} degrees"
            org = (50, 100)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, text, org, font, 1, (255, 0, 0), 2)

            height, width, _ = frame.shape

            # 세로 선 그리기
            line_color = (0, 255, 0)  # 초록색 (BGR 순서)
            line_thickness = 2  # 선 두께
            start_point = (math.trunc(width/2)+math.trunc(-angle), math.trunc(height/2)+100)  # 선의 시작점 (x, y)
            end_point = (math.trunc(width/2)+math.trunc(-angle), math.trunc(height/2)-100)  # 선의 끝점 (x, y)
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
            # 화면에 출력
            cv2.imshow('Steering Angle Detection', frame)

            # 'q' 키를 누를 때까지 비디오를 처리하며, 'q' 키를 누르면 루프 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # 비디오 캡처 객체 해제
    cap.release()

    # OpenCV 창 닫기
    cv2.destroyAllWindows()
