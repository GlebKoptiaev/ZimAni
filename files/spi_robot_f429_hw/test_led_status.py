"""Host tests for the exact pure LED decision functions extracted from main.c.

Run: python -m unittest -v test_led_status.py
Requires a host C compiler named gcc. No STM32 hardware is simulated.
"""
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest


def function_source(source, name):
    start = source.index('static uint32_t ' + name + '(')
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


@unittest.skipUnless(shutil.which('gcc'), 'Host gcc required')
class LedStatusTest(unittest.TestCase):
    def test_result_and_patterns(self):
        source = pathlib.Path(__file__).with_name('main.c').read_text()
        definitions = '\n'.join(re.findall(r'^#define LED_.*$', source, re.M))
        enums = '\n'.join(re.findall(r'enum \{ (?:LED_IDLE|STOP_WINDOW).*?\};', source, re.S))
        code = '#include <stdint.h>\n#include <assert.h>\n' + definitions + '\n' + enums
        code += '\n' + function_source(source, 'led_classify')
        code += '\n' + function_source(source, 'led_pattern')
        code += r'''
int main(void) {
    /* Missing data and CSN-only transitions must never turn success on. */
    assert(led_classify(0, STOP_WINDOW, 0, 0) == LED_EMPTY);
    assert(led_classify(0, STOP_WINDOW, 5, 0) == LED_EMPTY);
    assert(led_classify(0, STOP_WINDOW, 0, 8) == LED_EMPTY);
    assert(led_classify(0, STOP_WINDOW, 1, 1) == LED_OK);
    assert(led_classify(0, STOP_BYTES, 2, 32768) == LED_OK);
    assert(led_classify(0, STOP_FRAMES, 2048, 2048) == LED_OK);
    assert(led_classify(0, STOP_USER, 1, 4) == LED_OK);
    assert(led_classify(0, STOP_ERROR, 1, 4) == LED_ERROR);
    for (unsigned bit=0;bit<8;++bit)
        assert(led_classify(1u<<bit, STOP_WINDOW, 4, 32) == LED_ERROR);
    assert(led_pattern(LED_IDLE, 0) == LED_GREEN);
    assert(led_pattern(LED_IDLE, 100000) == 0);
    assert(led_pattern(LED_IDLE, 1000000) == LED_GREEN);
    assert(led_pattern(LED_BUSY, 500000) == LED_GREEN);
    assert(led_pattern(LED_OK, 500000) == LED_BLUE);
    assert(led_pattern(LED_OK, 1000) == (LED_GREEN|LED_BLUE));
    assert(led_pattern(LED_ERROR, 500000) == LED_RED);
    assert(led_pattern(LED_EMPTY, 0) == LED_GREEN);
    assert(led_pattern(LED_EMPTY, 499999) == 0);
    assert(led_pattern(LED_EMPTY, 500000) == LED_RED);
    assert(led_pattern(LED_EMPTY, 600000) == 0);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as folder:
            cfile = pathlib.Path(folder) / 'led_test.c'
            executable = pathlib.Path(folder) / 'led_test'
            cfile.write_text(code)
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            str(cfile), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    unittest.main()
