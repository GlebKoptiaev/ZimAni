"""Execute the firmware's actual pure boundary validator on the host."""
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest
from test_led_status import function_source


@unittest.skipUnless(shutil.which('gcc'), 'Host gcc required')
class BoundaryTests(unittest.TestCase):
    def test_valid_and_corrupt_boundaries(self):
        source = pathlib.Path(__file__).with_name('main.c').read_text()
        definitions = '\n'.join(re.findall(r'^#define (?:BYTE_CAP|FRAME_CAP).*$', source, re.M))
        enum = re.search(r'enum \{ F_ALIGN=.*?\};', source, re.S).group()
        code = '#include <stdint.h>\n#include <assert.h>\n' + definitions + '\n' + enum
        code += '\n' + function_source(source, 'validate_boundaries')
        code += r'''
int main(void) {
    uint32_t n=999;
    const uint32_t good[]={16,40}, empty[]={0,16,16};
    const uint32_t unaligned[]={15}, backwards[]={16,8}, maximum[]={262144};
    assert(validate_boundaries(good,2,5,&n)==0 && n==2);
    assert(validate_boundaries(empty,3,2,&n)==0 && n==3);
    assert(validate_boundaries(good,0,0,&n)==0 && n==0);
    assert(validate_boundaries(unaligned,1,2,&n)==F_ALIGN && n==0);
    assert(validate_boundaries(backwards,2,2,&n)==F_RANGE && n==1);
    assert(validate_boundaries(good,2,4,&n)==F_RANGE && n==1);
    assert(validate_boundaries(good,FRAME_CAP+1,5,&n)==F_RANGE && n==0);
    assert(validate_boundaries(good,2,BYTE_CAP+1,&n)==F_RANGE && n==0);
    assert(validate_boundaries(maximum,1,BYTE_CAP,&n)==0 && n==1);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as folder:
            cfile = pathlib.Path(folder) / 'boundary_test.c'
            binary = pathlib.Path(folder) / 'boundary_test'
            cfile.write_text(code)
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            str(cfile), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
