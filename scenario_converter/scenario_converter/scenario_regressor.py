import re


class Regressor():

    REPLACE_LOWER_PATTERN = re.compile(r"(^\w.*)")
    ARRANGE_EMPTY_PATTERN = re.compile(r"<([a-zA-Z]*)></\1>")
    ARRANGE_EMPTY_PATTERN2 = re.compile(r"<([a-zA-Z]*)(.*)></\1>")
    ARRANGE_DOUBLE_PATTERN = re.compile(r"(\s*)<(.[a-zA-Z]*)>\s*<\2>")
    UPSIDE_REPEAT_PATTERN = re.compile(r"<([a-zA-Z]*)( .*)>(\s+)</\1>")
    DOWNSIDE_REPEAT_PATTERN = re.compile(r"<([a-zA-Z]*)>(\s+)<\1( .*)>")
    UPPER_TRUE_PATTERN = re.compile(r"(True|\"True\")")
    UPPER_FALSE_PATTERN = re.compile(r"(False|\"False\")")

    def __init__(self):
        print("init")

    @staticmethod
    def apply_regression(text):
        text = Regressor.replace_empty_case(text)
        text = Regressor.replace_repeat_case(text)
        text = Regressor.replace_double_case(text)
        text = Regressor.replace_true_false_pattern(text)
        return text

    @staticmethod
    def replace_lower_case(phase):
        pattern = Regressor.REPLACE_LOWER_PATTERN
        repl = r"@\1"
        return re.sub(pattern, repl, phase)

    @staticmethod
    def replace_empty_case(phase):
        pattern = Regressor.ARRANGE_EMPTY_PATTERN
        repl = r"<\1/>"
        phase = re.sub(pattern, repl, phase)
        pattern = Regressor.ARRANGE_EMPTY_PATTERN2
        repl = r"<\1\2/>"
        return re.sub(pattern, repl, phase)

    @staticmethod
    def replace_double_case(phase):
        pattern = Regressor.ARRANGE_DOUBLE_PATTERN
        repl = r"\1<\2>"
        return re.sub(pattern, repl, phase, re.X)

    @staticmethod
    def replace_repeat_case(phase):
        pattern = Regressor.UPSIDE_REPEAT_PATTERN
        repl = r"<\1\3>"
        phase = re.sub(pattern, repl, phase)
        pattern = Regressor.DOWNSIDE_REPEAT_PATTERN
        repl = r"<\1\2>"
        return re.sub(pattern, repl, phase)

    @staticmethod
    def replace_true_false_pattern(phase):
        pattern = Regressor.UPPER_TRUE_PATTERN
        repl = r'"true"'
        phase = re.sub(pattern, repl, phase)
        pattern = Regressor.UPPER_FALSE_PATTERN
        repl = r'"false"'
        return re.sub(pattern, repl, phase)


if __name__ == "__main__":
    empty_case_test = Regressor.replace_empty_case("<tag></tag>")
    lower_case_test = Regressor.replace_lower_case("data")
    print("empty_case:", empty_case_test)
    print("lower case:", lower_case_test)
    double_case_test = Regressor.replace_double_case("</tag>\n\t</tag>")
    print("double:", double_case_test)
