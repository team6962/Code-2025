import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Define the floors as y-values:
floors = [0, 1, 2]

class Stage:
    height: int
    color: str
    loc: int

    def __init__(self, height, color):
        self.loc = 0
        self.color = color
        self.height = height

class Elevator:
    stages: list[Stage] = []
    stage_height: int
    rects: list[plt.Rectangle] = []

    def __init__(self, stage_count, stage_height):
        self.stage_height = stage_height
        colors = ['red', 'green', 'blue', 'yellow', 'purple', 'orange']
        for i in range(stage_count):
            self.stages.append(Stage(stage_height, colors[i % len(colors)] ))
            self.rects.append(plt.Rectangle((0, 0), 0.1, stage_height, fc=self.stages[i].color))

    def move(self, direction):
        for i in range(len(self.stages)):
            # prev_stage = self.stages[i-1] if i > 0 else None
            # stage = self.stages[i]
            # stage.loc = (prev_stage.loc if prev_stage else 0) + self.stage_height + direction

            self.stages[i].loc += direction * i

    def draw(self):
        for rect, stage in zip(self.rects, self.stages):
            rect.set_y(stage.loc)
        return self.rects

# Create a figure and axis:
fig, ax = plt.subplots(figsize=(4,6))

# # Plot horizontal lines for each floor:
# for f in floors:
#     ax.hlines(y=f, xmin=-1, xmax=1, color='gray', linewidth=1, linestyles='dashed')
#     ax.text(1.1, f, f'Floor {f}', va='center', fontsize=10)




# Set plot limits:
ax.set_xlim(-1, 2)
ax.set_ylim(-0.5, 2.5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('3-Stage Elevator Visualization')

elevator = Elevator(3, 1)
ax.add_patch(elevator.rects[0])
ax.add_patch(elevator.rects[1])
ax.add_patch(elevator.rects[2])

# Animation update function:
# Here, we make the elevator move smoothly from y=0 to y=2 over 100 frames
def update(frame):
    # frame will go from 0 to 99
    t = frame / 99.0  # normalize 0 → 1
    elevator.move(0.01)
    return elevator.draw()

# Create the animation:
ani = animation.FuncAnimation(fig, update, frames=100, interval=50, blit=True)

plt.show()
