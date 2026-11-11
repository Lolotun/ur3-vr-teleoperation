import socket
import threading
import time
import unittest
from teleop_core.robotiq_feedback import parse_register, read_feedback


class FeedbackTests(unittest.TestCase):
    def test_register_response_validation(self):
        self.assertEqual(parse_register('POS', b'POS 128\n'), 128)
        for reply in (b'PRE 128', b'POS 256', b'POS -1', b'ack', b'POS nan', b'POS 1 garbage'):
            with self.subTest(reply=reply), self.assertRaises(ValueError):
                parse_register('POS', reply)

    def test_get_only_contact_and_position_differ_from_requested(self):
        commands = []
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0)); server.listen()
            def serve():
                connection, _ = server.accept()
                with connection, connection.makefile('rb') as lines:
                    for line in lines:
                        command = line.decode().strip(); commands.append(command)
                        variable = command.split()[1]
                        value = {'POS': 128, 'PRE': 255, 'OBJ': 2, 'STA': 3}[variable]
                        connection.sendall(f'{variable} {value}\n'.encode())
            thread = threading.Thread(target=serve); thread.start()
            result = read_feedback('127.0.0.1', server.getsockname()[1], clock_ns=lambda: 123)
            thread.join(timeout=1)
        self.assertEqual(commands, ['GET POS', 'GET PRE', 'GET OBJ', 'GET STA'])
        self.assertEqual(result['stamp_ns'], 123)
        self.assertAlmostEqual(result['position_norm'], 128/255)
        self.assertEqual(result['requested_norm'], 1.)
        self.assertTrue(result['contact_detected'])
        self.assertFalse(result['moving'])
        self.assertFalse(result['at_destination'])

    def test_read_timeout_is_bounded(self):
        with socket.socket() as server:
            server.bind(('127.0.0.1',0));server.listen()
            def serve():
                connection, _ = server.accept()
                with connection:
                    connection.recv(100);time.sleep(.15)
            thread = threading.Thread(target=serve);thread.start()
            start = time.monotonic()
            with self.assertRaises(TimeoutError):
                read_feedback('127.0.0.1',server.getsockname()[1],timeout=.04)
            elapsed = time.monotonic()-start
            thread.join(timeout=1)
        self.assertLess(elapsed,.12)


if __name__ == '__main__':
    unittest.main()
