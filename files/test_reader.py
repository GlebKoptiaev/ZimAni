import unittest
from reader import parse_capture,decode_frame

class DecoderTests(unittest.TestCase):
    def test_rx_excludes_status(self):
        r=decode_frame(0,100,bytes.fromhex('61FFFFFF'),bytes.fromhex('40112233'))
        self.assertEqual(r['status'],'40')
        self.assertEqual(r['data_hex'],'112233')
        self.assertEqual(r['direction'],'radio_to_robot')
    def test_register_and_tx(self):
        r=decode_frame(0,1,bytes.fromhex('252A'),bytes.fromhex('0E00'))
        self.assertEqual(r['register'],'RF_CH')
        self.assertIn('2442',r['note'])
        r=decode_frame(0,1,bytes.fromhex('A01122'),bytes.fromhex('0E0000'))
        self.assertEqual(r['data_hex'],'1122')
    def test_ack(self):
        r=decode_frame(0,1,bytes.fromhex('AABBCC'),bytes.fromhex('0E0000'))
        self.assertEqual(r['command'],'W_ACK_PAYLOAD')
        self.assertEqual(r['data_hex'],'BBCC')
        self.assertEqual(r['note'],'pipe=2')
    def test_good_capture(self):
        s='#BEGIN version=1 reason=WINDOW flags=0x0 frames=1 complete_bytes=3\nF,0,15,61FFFF,400102\n#END\n'
        self.assertEqual(parse_capture(s)[1][0]['data_hex'],'0102')
    def test_reject_error_and_incomplete(self):
        s='#BEGIN version=1 reason=ERROR flags=0x1 frames=0 complete_bytes=0\n#END\n'
        with self.assertRaises(ValueError):parse_capture(s)
        self.assertEqual(parse_capture(s,True)[1],[])
        with self.assertRaises(ValueError):parse_capture(s.replace('#END',''))
    def test_reject_missing_data(self):
        s='#BEGIN version=1 reason=WINDOW flags=0x0 frames=1 complete_bytes=4\nF,0,15,61FFFF,400102\n#END\n'
        with self.assertRaises(ValueError):parse_capture(s)
    def test_reject_unequal_channels(self):
        with self.assertRaises(ValueError):decode_frame(0,0,b'\x61',b'')

if __name__=='__main__':unittest.main()
