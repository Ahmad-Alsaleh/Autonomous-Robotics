from controller import AnsiCodes


class Logger:
    blue = AnsiCodes.BLUE_FOREGROUND
    yellow = AnsiCodes.YELLOW_FOREGROUND
    red = AnsiCodes.RED_FOREGROUND
    bold_red = AnsiCodes.BOLD + AnsiCodes.RED_FOREGROUND
    reset = AnsiCodes.RESET
    green = AnsiCodes.GREEN_FOREGROUND

    @classmethod
    def info(cls, message):
        print(cls.blue + message + cls.reset)

    @classmethod
    def warning(cls, message):
        print(cls.yellow + message + cls.reset)

    @classmethod
    def error(cls, message):
        print(cls.red + message + cls.reset)

    @classmethod
    def critical(cls, message):
        print(cls.bold_red + message + cls.reset)

    @classmethod
    def success(cls, message):
        print(cls.green + message + cls.reset)
