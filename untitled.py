import pygame
import sys

# 初始化Pygame
pygame.init()

# 设置窗口大小
window_size = (900, 600)

# 创建窗口
screen = pygame.display.set_mode(window_size)
pygame.display.set_caption('Moving Circle')

# 设置圆的初始位置和速度
circle_radius = 20
circle_color = (255, 0, 0)
circle_position = [window_size[0] // 2, window_size[1] // 2]
circle_speed = [5, 5]

# 加载显示坐标的字体
font = pygame.font.Font(None, 36)

# # 图片路径
# image_path = 'path/to/your/image.png'
# # 加载图像
# image = pygame.image.load(image_path)
# image = pygame.transform.scale(image, (50, 50))

# 游戏主循环
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # 移动圆的位置
    circle_position[0] += circle_speed[0]
    circle_position[1] += circle_speed[1]

    # 碰撞检测，反向移动圆的速度
    if circle_position[0] <= 0 or circle_position[0] >= window_size[0]:
        circle_speed[0] = -circle_speed[0]
    if circle_position[1] <= 0 or circle_position[1] >= window_size[1]:
        circle_speed[1] = -circle_speed[1]

    # 清除屏幕
    screen.fill((255, 255, 255))

    # 画圆
    pygame.draw.circle(screen, circle_color, (int(circle_position[0]), int(circle_position[1])), circle_radius)

    # 显示当前坐标
    text = font.render(f'Coordinates: ({circle_position[0]}, {circle_position[1]})', True, (0, 0, 0))
    screen.blit(text, (window_size[0] - 300, 20))

    # # 显示图像
    # screen.blit(image, (window_size[0] - 100, window_size[1] // 2 - 25))

    # 刷新屏幕
    pygame.display.flip()

    # 控制帧率
    pygame.time.Clock().tick(60)
