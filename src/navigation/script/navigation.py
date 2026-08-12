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

    # Ждём активации Nav2
    navigator.waitUntilNav2Active()


    # # Задаём список waypoints (координаты в системе 'map') - малый круг Н29
    # keypoints = [
    #     create_pose(navigator, 1.273, -1.337, 0.062),   # 
    #     create_pose(navigator, 4.233, 7.501, 1.497),    # 
    #     create_pose(navigator, 1.449, 14.256, -3.129),     # 
    #     create_pose(navigator, -1.089, 6.683, -1.615)     # 
    # ]
    # keypoints *= 10

    # keypoints.append(keypoints[0])

    # Openspace Дорожный (часть)
    keypoints = [
        create_pose(navigator, 4.135, 0.202, 1.073),   
        create_pose(navigator, 1.883, 0.227, 1.091),   
        create_pose(navigator, 0.083, 0.203, 1.094),   
        create_pose(navigator, -1.421, 0.200, 1.085),   
        create_pose(navigator, -3.750, 0.171, 1.090),   
        create_pose(navigator, -5.336, 0.013, 1.046),   
        create_pose(navigator, -8.223, 0.019, 1.056),   
        create_pose(navigator, -9.464, -0.040, 1.058),   
        create_pose(navigator, -8.860, -3.164, -1.044),   
        create_pose(navigator, 0.0, 0.0, 0.0)   
    ]




    # for i in range(2):        
    navigator.followWaypoints(keypoints)
    # navigator.goThroughPoses(keypoints)
    # Мониторим выполнение
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # Можно добавить логику по времени/расстоянию
        time.sleep(3)


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