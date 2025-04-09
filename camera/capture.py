import cv2

# 打开摄像头
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


while True:
    # 读取摄像头的一帧画面
    ret, frame = cap.read()

    if not ret:
        print("无法获取画面")
        break

    # 显示画面
    cv2.imshow('Camera', frame)

    # 按 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # 按 's' 键拍照
    elif cv2.waitKey(1) & 0xFF == ord('s'):
        cv2.imwrite('captured_photo.jpg', frame)
        print("照片已保存为 captured_photo.jpg")

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()
    