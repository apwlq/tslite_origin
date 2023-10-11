import math
import cv2
import numpy as np

def detect_steering_angle(frame):
    # 중앙에 가로로 긴 직사각형 영역(사각 박스)을 생성합니다.
    height, width, _ = frame.shape
    roi_width = int(width * 0.6)
    roi_height = int(height * 0.2)
    roi_x = int((width - roi_width) / 2)
    roi_y = int(height * 0.4)

    # 마스크를 생성하여 사각 박스 영역만 남깁니다.
    mask = np.zeros_like(frame)
    cv2.rectangle(mask, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (255, 255, 255), thickness=cv2.FILLED)
    roi_frame = cv2.bitwise_and(frame, mask)

    # 그레이스케일 변환
    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)

    # 캐니 엣지 검출
    # edges = cv2.Canny(gray, 50, 150)
    edges = cv2.Canny(gray, 400, 400)

    cv2.imshow('edges', edges)

    # 허프 변환을 사용하여 선을 검출합니다.
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=50)

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
        center_y = (y1 + y2) / 2

        # 중앙보다 왼쪽에 있는 경우
        if center_x < width / 2:
            left_lines.append(line[0])
        # 중앙보다 오른쪽에 있는 경우
        else:
            right_lines.append(line[0])

    # Check if there are lines on both sides
    if not left_lines or not right_lines:
        return None

    # 왼쪽과 오른쪽 선의 중점을 계산하여 각도를 추출합니다.
    left_center = np.mean(left_lines, axis=0)
    right_center = np.mean(right_lines, axis=0)

    # 두 중점 사이의 거리를 계산하여 각도 추출
    angle = np.arctan2(right_center[1] - left_center[1], right_center[0] - left_center[0])

    # 각도를 라디안에서 도로 변환
    angle_degrees = np.degrees(angle)

    # 마스크 박스와 선을 프레임에 그립니다.
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (255, 0, 0), 2)  # 마스크 박스
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 선

    return angle_degrees

if __name__ == "__main__":
    video_path = "videos.mkv"  # 분석할 비디오 파일 경로를 여기에 입력하세요.
    cap = cv2.VideoCapture(1)

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