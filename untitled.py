import pygame
import sys

# Initialize Pygame
pygame.init()

# Set up display
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Mouse Click Example")

# Set up colors
white = (255, 255, 255)

# Main game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left mouse button
                print("Left mouse button clicked at", event.pos)
            elif event.button == 3:  # Right mouse button
                print("Right mouse button clicked at", event.pos)

    # Fill the screen with white
    screen.fill(white)

    # Update the display
    pygame.display.flip()

# Quit Pygame
pygame.quit()
sys.exit()
