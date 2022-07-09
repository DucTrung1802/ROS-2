from oled_display.Oled import Oled


def main():
    oled = Oled()
    oled.add_text(text="helloooooooooooo")
    oled.display()
