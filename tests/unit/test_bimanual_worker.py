"""Pure-memory receipt deadlines; no ROS, solver process, or device endpoints."""
import queue
import ast
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from bindu_kinematics.bimanual_worker import BimanualWorker


class WorkerReceiptTests(unittest.TestCase):
    def worker(self, *, ready=True, busy=True):
        worker = BimanualWorker.__new__(BimanualWorker)
        worker.requests, worker.results = queue.Queue(1), queue.Queue(2)
        worker.process = SimpleNamespace(is_alive=lambda:True)
        worker.ready, worker.busy = ready, busy
        worker.error = ''
        worker.failure = ''
        worker.diagnostic = None
        worker.started = 0.
        worker.submitted = 10.
        worker.startup_timeout = 2.
        worker.timeout = .23
        return worker

    def poll(self, worker, now):
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=now):
            return worker.poll()

    def test_queued_late_success_and_rejection_cannot_erase_request_deadline(self):
        for kind in ('result', 'error'):
            with self.subTest(kind=kind):
                worker = self.worker()
                worker.results.put((kind, 7, 'late'))
                with self.assertRaisesRegex(RuntimeError, '^BIMANUAL_WORKER_TIMEOUT$'):
                    self.poll(worker, 10.231)
                self.assertFalse(worker.ready)
                self.assertFalse(worker.submit('solve', 8, {}))
                self.assertTrue(worker.requests.empty())

    def test_late_ready_cannot_erase_startup_deadline(self):
        worker = self.worker(ready=False, busy=False)
        worker.results.put(('ready', 0, None))
        with self.assertRaisesRegex(RuntimeError, '^BIMANUAL_WORKER_TIMEOUT$'):
            self.poll(worker, 2.001)

    def test_timeout_stays_latched_when_late_result_arrives(self):
        worker = self.worker()
        with self.assertRaisesRegex(RuntimeError, '^BIMANUAL_WORKER_TIMEOUT$'):
            self.poll(worker, 10.231)
        worker.results.put(('result', 7, 'late'))
        for now in (10.24, 11., 100.):
            with self.assertRaisesRegex(RuntimeError, '^BIMANUAL_WORKER_TIMEOUT$'):
                self.poll(worker, now)
        self.assertEqual(worker.results.qsize(), 1)

    def test_in_budget_diagnostic_and_result_are_delivered_once(self):
        worker = self.worker()
        worker.results.put(('diagnostic', 7, {'scale':.5}))
        worker.results.put(('result', 7, 'accepted-candidate'))
        self.assertEqual(self.poll(worker, 10.22), ('result', 7, 'accepted-candidate'))
        self.assertEqual(worker.diagnostic, (7, {'scale':.5}))
        self.assertFalse(worker.busy)
        # Idle time does not expire a completed request.
        self.assertIsNone(self.poll(worker, 100.))

    def test_in_budget_operation_error_is_not_a_worker_fault(self):
        worker = self.worker()
        worker.results.put(('error', 7, 'QP_INFEASIBLE'))
        self.assertEqual(self.poll(worker, 10.1), ('error', 7, 'QP_INFEASIBLE'))
        self.assertTrue(worker.ready)
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=10.11):
            self.assertTrue(worker.submit('capture', 8, {}))

    def test_startup_and_request_have_separate_receipt_budgets(self):
        worker = self.worker(ready=False, busy=False)
        worker.results.put(('ready', 0, None))
        self.assertIsNone(self.poll(worker, 1.99))
        self.assertTrue(worker.ready)
        self.assertIsNone(self.poll(worker, 50.))
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=50.):
            self.assertTrue(worker.submit('capture', 1, {}))
        worker.results.put(('diagnostic', 1, {'scale':1.}))
        self.assertIsNone(self.poll(worker, 50.1))
        self.assertTrue(worker.busy)
        with self.assertRaisesRegex(RuntimeError, 'WORKER_TIMEOUT'):
            self.poll(worker, 50.24)

    def test_runtime_fault_emits_once_and_continues_waking_stop_waiters(self):
        source = Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/bimanual_tool_node.py'
        node = next(n for n in ast.parse(source.read_text()).body
                    if isinstance(n, ast.ClassDef) and n.name == 'BimanualToolNode')
        method = next(n for n in node.body if getattr(n, 'name', '') == 'poll_worker')
        namespace = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
        worker = self.worker()
        owner = SimpleNamespace(waiters=set(), worker_error='', worker=worker, event=Mock())
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=10.24):
            namespace['poll_worker'](owner)
            worker.results.put(('result', 7, 'late'))
            for _ in range(1000):
                waiter = Mock()
                waiter.done.return_value = False
                owner.waiters = {waiter}
                namespace['poll_worker'](owner)
                waiter.set_result.assert_called_once_with(None)
        owner.event.assert_called_once_with('BIMANUAL_WORKER_FAILED', 'BIMANUAL_WORKER_TIMEOUT')
        self.assertEqual(worker.results.qsize(), 1)

    def test_dead_process_or_fatal_cannot_advertise_ready_or_accept_work(self):
        for cause in ('exit', 'fatal'):
            with self.subTest(cause=cause):
                worker = self.worker(busy=False)
                code = 'BIMANUAL_WORKER_EXITED'
                if cause == 'exit':
                    worker.process = SimpleNamespace(is_alive=lambda:False)
                else:
                    worker.results.put(('fatal', 0, 'ValueError: MODEL_INVALID'))
                    code = 'BIMANUAL_WORKER_FAILED:ValueError: MODEL_INVALID'
                for now in (10., 10.1):
                    with self.assertRaisesRegex(RuntimeError, '^'+code+'$'):
                        self.poll(worker, now)
                    self.assertFalse(worker.ready)
                    self.assertFalse(worker.submit('capture', 8, {}))

    def test_long_jittered_sequence_has_no_replayed_or_resurrected_result(self):
        worker = self.worker(busy=False)
        # Simulated 10,000 bounded requests, with dropped polls and jitter.
        for token in range(10000):
            start = 10.+token*.3
            with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=start):
                self.assertTrue(worker.submit('solve', token, {}))
            self.assertEqual(worker.requests.get_nowait()[1], token)
            self.assertIsNone(self.poll(worker, start+.01))
            worker.results.put(('result', token, token))
            self.assertEqual(self.poll(worker, start+.02+(token%7)*.025), ('result', token, token))
            self.assertIsNone(self.poll(worker, start+.21))
        # A lost response then arriving after the budget must stay unusable.
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=4000.):
            self.assertTrue(worker.submit('solve', 10000, {}))
        with self.assertRaisesRegex(RuntimeError, 'WORKER_TIMEOUT'):
            self.poll(worker, 4000.24)
        worker.results.put(('result', 10000, 'late'))
        with self.assertRaisesRegex(RuntimeError, 'WORKER_TIMEOUT'):
            self.poll(worker, 4000.25)


if __name__ == '__main__':
    unittest.main()
