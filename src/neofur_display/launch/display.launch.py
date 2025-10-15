import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取 neofur_display 包的 share 目录路径
    #    这个路径会指向 .../install/neofur_display/share/neofur_display
    pkg_share_path = get_package_share_directory('neofur_display')

    # 2. 根据你的 CMakeLists.txt 安装规则，资源文件位于 share 目录下的 resource/ 文件夹
    #    我们在 launch 文件中构建这个基础路径
    image_base_path = os.path.join(pkg_share_path, 'resource','eyes')

    # 3. 定义并启动节点
    neofur_display_node = Node(
        package='neofur_display',
        executable='neofur_display_node',
        name='neofur_display_node',
        output='screen',
        emulate_tty=True, # 确保日志能实时打印到终端
        parameters=[
            # 将我们刚刚构建好的路径作为参数 'image_base_path' 传递给节点
            {'image_base_path': image_base_path},

            # 其他参数也可以在这里设置
            {'drm_device': '/dev/dri/card0'},
            {'image_name': 'green_left_571x620.rgba'}
        ]
    )

    return LaunchDescription([
        neofur_display_node
    ])
