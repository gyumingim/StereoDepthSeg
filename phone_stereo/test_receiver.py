"""Protocol corruption, partial USB reads, dropped frames, and calibration mixups."""
import json
import tempfile
from pathlib import Path
import unittest

import numpy as np

from lib_phone_stereo import HEADER, MAGIC, PhoneFrame, FramePairer, read_frame
from lib_phone_depth import load_phone_calibration


class FragmentedSocket:
    def __init__(self, data):
        self.data = bytearray(data)

    def recv_into(self, target):
        n = min(3, len(target), len(self.data))
        target[:n] = self.data[:n]
        del self.data[:n]
        return n


def frame(index, milliseconds):
    return PhoneFrame(index, int(milliseconds * 1e6), np.zeros((2, 2, 3), np.uint8))


class ReceiverTests(unittest.TestCase):
    def test_fragmented_packet_and_nv21_chroma(self):
        # Dark luma + neutral VU -> black BGR, despite arbitrarily split TCP reads.
        data = HEADER.pack(MAGIC, 1, 2, 2, 9876543210, 6) + bytes([16]*4+[128,128])
        f = read_frame(FragmentedSocket(data), 2, (2,2))
        self.assertEqual((f.index, f.timestamp_ns), (1,9876543210))
        np.testing.assert_array_equal(f.bgr, np.zeros((2,2,3),np.uint8))

    def test_truncated_payload_fails(self):
        data = HEADER.pack(MAGIC, 0, 2, 2, 1, 6) + bytes(3)
        with self.assertRaises(EOFError):
            read_frame(FragmentedSocket(data),2,(2,2))

    def test_corrupt_header_rejected_before_allocation(self):
        for magic,index,w,h,n in [(MAGIC,2,2,2,6),(0,0,2,2,6),(MAGIC,0,20000,20000,600000000),
                                (MAGIC,0,2,2,5)]:
            with self.subTest(index=index,w=w,n=n), self.assertRaises(ValueError):
                read_frame(FragmentedSocket(HEADER.pack(magic,index,w,h,1,n)),2,(2,2))

    def test_pair_drops_old_frame_and_never_reuses(self):
        p = FramePairer(2,2)
        self.assertIsNone(p.add(frame(0,100)))
        self.assertIsNone(p.add(frame(1,134)))
        pair = p.add(frame(0,133))
        self.assertEqual([f.timestamp_ns for f in pair],[133000000,134000000])
        self.assertEqual(p.dropped,[1,0])
        self.assertIsNone(p.add(frame(0,167)))

    def test_three_streams_require_all_three(self):
        p = FramePairer(3,1)
        self.assertIsNone(p.add(frame(2,20)))
        self.assertIsNone(p.add(frame(0,20)))
        self.assertEqual([f.index for f in p.add(frame(1,20))],[0,1,2])
        self.assertIsNone(p.add(frame(1,21)))

    def test_missing_camera_queue_is_bounded(self):
        p=FramePairer(2,0)
        for i in range(100):
            self.assertIsNone(p.add(frame(0,i)))
        self.assertEqual(len(p.queues[0]),8)
        self.assertEqual(p.dropped,[92,0])

    def test_calibration_rejects_other_lenses_and_size(self):
        st=dict(phone=dict(logical="0",physical=["5","2"]), image_size=[640,480],
                K1=np.eye(3).tolist(),K2=np.eye(3).tolist(),dist1=[0]*5,dist2=[0]*5,
                R=np.eye(3).tolist(),T=[-.016,0,0])
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"calibration.json"
            path.write_text(json.dumps(st))
            self.assertEqual(load_phone_calibration(path,"0",["5","2"],[640,480]),(st,False))
            # 같은 두 렌즈를 반대 순서로 받으면 거부하지 않고 swap=True (수신 쌍을 뒤집어 쓴다)
            self.assertEqual(load_phone_calibration(path,"0",["2","5"],[640,480]),(st,True))
            self.assertEqual(load_phone_calibration(path,"0",["2","5","6"],[640,480]),(st,True))
            for physical,size in [(["5","2"],[1280,720]),(["5","6"],[640,480]),(["2","6"],[640,480])]:
                with self.assertRaises(ValueError):
                    load_phone_calibration(path,"0",physical,size)
            del st["phone"]
            path.write_text(json.dumps(st))
            with self.assertRaises(ValueError):
                load_phone_calibration(path,"0",["5","2"],[640,480])


if __name__=="__main__":
    unittest.main()
