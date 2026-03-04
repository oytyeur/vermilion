from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
import time

def create_pose(navigator, x, y, yaw, frame_id='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = yaw  # упрощённо (для реального кода используйте quaternion)
    return pose

def main():
    rclpy.init()

    # Создаём навигатор
    navigator = BasicNavigator()

    # # Устанавливаем начальную позицию (если нужно)
    # initial_pose = create_pose(0.0, 0.0, 0.0)
    # navigator.setInitialPose(initial_pose)

    # Ждём активации Nav2
    navigator.waitUntilNav2Active()


    # # Задаём список waypoints (координаты в системе 'map')
    keypoints = [
        create_pose(navigator, 1.273, -1.337, 0.062),   # 
        create_pose(navigator, 4.233, 7.501, 1.497),    # 
        create_pose(navigator, 1.449, 14.256, -3.129),     # 
        create_pose(navigator, -1.089, 6.683, -1.615)     # 
    ]

    # keypoints *= 3

    for i in range(3):
        # Запускаем прохождение waypoints
        # navigator.followWaypoints(keypoints)
        navigator.goThroughPoses(keypoints)
        # Мониторим выполнение
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            # Можно добавить логику по времени/расстоянию
            time.sleep(1)


    # Проверяем результат
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Все waypoints пройдены!')
    elif result == TaskResult.CANCELED:
        print('Задача отменена.')
    else:
        print('Ошибка при прохождении waypoints.')


    rclpy.shutdown()

if __name__ == '__main__':
    main()