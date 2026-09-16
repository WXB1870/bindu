"""Nonblocking sockets, owned exclusively by the caller's thread."""
import time
import zmq
from .protocol import decode, encode, MAX_BYTES


class PiTransport:
    def __init__(self, config):
        self.config = config
        self.context = zmq.Context()
        self.sockets = []
        self.seq = 0
        self.next_publish = 0.
        try:
            self.state = self._socket(zmq.PUB if config['mode'] == 'pubsub' else zmq.REP)
            self.state.bind(config['state_bind'])
            self.commands = self._socket(zmq.SUB)
            self.commands.setsockopt_string(zmq.SUBSCRIBE, config['command_topic'])
            self.commands.connect(config['command_connect'])
        except Exception:
            self.close()
            raise

    def _socket(self, kind):
        sock = self.context.socket(kind)
        self.sockets.append(sock)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.SNDHWM, 2)
        sock.setsockopt(zmq.RCVHWM, 2)
        sock.setsockopt(zmq.MAXMSGSIZE, MAX_BYTES)
        return sock

    def exchange(self, payload):
        """Serve a snapshot or an explicit not-ready response, then read commands.

        Returns (messages, diagnostics); bounded drain never blocks execution.
        Command eligibility and observation identity belong to the adapter.
        """
        diagnostics, messages = [], []
        extra = None
        send = False
        if self.config['mode'] == 'pubsub':
            if payload is not None and time.monotonic() >= self.next_publish:
                send = True
                self.next_publish = time.monotonic() + 1. / self.config['state_rate_hz']
        elif self.state.poll(0, zmq.POLLIN):
            received = time.time()
            frames = self.state.recv_multipart(zmq.NOBLOCK)
            try:
                request = decode(frames)
                if request.topic != self.config['request_topic']:
                    raise ValueError('PI_REQUEST_TOPIC')
                extra = {'request_seq': request.seq, 'request_timestamp_send': request.timestamp_send,
                         'request_received_wall': received}
                payload = ({**payload, 'ok': True, 'request_seq': request.seq,
                            'request_received_wall': received, 'state_reply_ready_wall': time.time()}
                           if payload is not None else {'ok': False, 'error': 'PI_OBSERVATION_NOT_READY'})
            except ValueError as exc:
                diagnostics.append(str(exc))
                payload = {'ok': False, 'error': str(exc)}
            send = True  # REP must reply even to invalid requests.
        if send:
            self.state.send_multipart(encode(self.config['state_topic'], self.seq, time.time(), payload, extra), zmq.NOBLOCK)
            self.seq += 1
        for _ in range(32):
            try:
                frames = self.commands.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
            try:
                msg = decode(frames)
                if msg.topic != self.config['command_topic']:
                    raise ValueError('PI_COMMAND_TOPIC')
                messages.append(msg)
            except ValueError as exc:
                diagnostics.append(str(exc))
        return messages, diagnostics

    def close(self):
        for sock in self.sockets:
            sock.close(0)
        self.context.term()
