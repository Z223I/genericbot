from MuleBot import MuleBot as ChickenBot

bot = ChickenBot()

def f(speed):
    print(f"Running 'f' method.")
    bot.motorSpeed(speed, speed)


def forward(inches):
    bot.forward(inches)
    print(f"ChickBot moved forward {inches} inches.")

def backward(inches):
    bot.backward(inches)
    print(f"ChickBot moved backward {inches} inches.")

def back(inches):
    bot.backward(inches)
    print(f"ChickBot moved backward {inches} inches.")

def right(degrees):
    bot.turn('right', degrees)
    print(f"ChickBot turned right {degrees} degrees")

def left(degrees):
    bot.turn('left', degrees)
    print(f"ChickBot turned left {degrees} degrees")
