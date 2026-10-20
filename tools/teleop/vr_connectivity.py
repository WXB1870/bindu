#!/usr/bin/env python3
"""Test the existing Vuer controller receiver without ROS, IK or device commands."""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import ssl
import sys
import time

from tools import ROOT
for package in ('shared/bindu_contracts', 'capabilities/bindu_teleoperation'):
    sys.path.insert(0, str(ROOT/'src'/package))


def create_certificates(directory, address):
    """Create a site-specific CA and leaf; never install trust or replace keys."""
    from datetime import datetime, timedelta, timezone
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.x509.oid import NameOID, ExtendedKeyUsageOID
    ip = ipaddress.ip_address(address)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    paths = [directory/name for name in ('rootCA.key', 'rootCA.crt', 'server.key', 'server.crt')]
    if any(p.exists() for p in paths):
        raise ValueError('证书文件已存在，请复用或选择另一目录；不会覆盖。')
    now = datetime.now(timezone.utc)
    ca_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, 'Bindu VR local test CA')])
    ca = (x509.CertificateBuilder().subject_name(name).issuer_name(name)
          .public_key(ca_key.public_key()).serial_number(x509.random_serial_number())
          .not_valid_before(now-timedelta(minutes=5)).not_valid_after(now+timedelta(days=365))
          .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
          .add_extension(x509.KeyUsage(False, False, False, False, False, True, True, False, False), critical=True)
          .sign(ca_key, hashes.SHA256()))
    cert = (x509.CertificateBuilder()
            .subject_name(x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, address)]))
            .issuer_name(name).public_key(key.public_key()).serial_number(x509.random_serial_number())
            .not_valid_before(now-timedelta(minutes=5)).not_valid_after(now+timedelta(days=90))
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .add_extension(x509.SubjectAlternativeName([x509.IPAddress(ip),
                x509.IPAddress(ipaddress.ip_address('127.0.0.1')), x509.DNSName('localhost')]), critical=False)
            .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
            .add_extension(x509.KeyUsage(True, False, True, False, False, False, False, False, False), critical=True)
            .sign(ca_key, hashes.SHA256()))
    values = [ca_key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                  serialization.NoEncryption()), ca.public_bytes(serialization.Encoding.PEM),
              key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                serialization.NoEncryption()), cert.public_bytes(serialization.Encoding.PEM)]
    for path, value in zip(paths, values):
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600 if path.suffix == '.key' else 0o644)
        with os.fdopen(fd, 'wb') as stream:
            stream.write(value)
    print('已生成站点测试证书。只向头显导入公开 CA：'+str(paths[1].resolve()))
    print('头显须信任该 CA；生成文件不表示头显已信任。')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--advertise-host', help='头显能访问的电脑局域网 IP')
    parser.add_argument('--port', type=int, default=8012)
    parser.add_argument('--cert-file')
    parser.add_argument('--key-file')
    parser.add_argument('--duration', type=float, default=300., help='运行秒数；0 为持续运行')
    parser.add_argument('--make-cert', metavar='DIR', help='生成局域网测试证书后退出；需 cryptography')
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 0 <= args.duration < float('inf'):
        parser.error('端口或运行时长无效')
    try:
        if args.make_cert:
            if not args.advertise_host:
                parser.error('--make-cert 需要 --advertise-host 指定局域网 IP')
            create_certificates(args.make_cert, args.advertise_host)
            return 0
        if bool(args.cert_file) != bool(args.key_file):
            parser.error('--cert-file 和 --key-file 必须同时提供')
        if args.host not in ('127.0.0.1', 'localhost', '::1') and not args.cert_file:
            parser.error('头显局域网接入需要 HTTPS 证书和私钥')
        if args.host in ('0.0.0.0', '::') and not args.advertise_host:
            parser.error('监听全部接口时需要 --advertise-host 指定头显访问地址')
        if args.cert_file:
            ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER).load_cert_chain(args.cert_file, args.key_file)
        from bindu_teleoperation.vr.connectivity import VRConnectivity
        from bindu_teleoperation.vr.receiver import VuerReceiver
        config = dict(host=args.host, port=args.port, side='both', mapping='pico_snowbot_v1',
                      cert_file=args.cert_file, key_file=args.key_file, connectivity_check=True)
        receiver = VuerReceiver(config)
        scheme, ws_scheme = ('https', 'wss') if args.cert_file else ('http', 'ws')
        address = args.advertise_host or args.host
        if ':' in address:
            address = '['+address+']'
        endpoint = f'{address}:{args.port}/'
        print(f'头显浏览器访问：{scheme}://{endpoint}?ws={ws_scheme}://{endpoint}', flush=True)
        print('仅接收手柄输入；未导入 ROS、IK 或机器人驱动。Ctrl+C 退出。', flush=True)
        monitor = VRConnectivity()
        start = time.monotonic()
        next_report = start
        try:
            while args.duration == 0 or time.monotonic()-start < args.duration:
                frames = receiver.poll()
                if frames is not None:
                    monitor.ingest(frames)
                now = time.monotonic()
                if now >= next_report:
                    print(json.dumps(monitor.snapshot(time.time()), ensure_ascii=False), flush=True)
                    next_report = now+1.
                time.sleep(.005)
        finally:
            receiver.close()
        return 0
    except KeyboardInterrupt:
        return 0
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        print('VR 连通检查失败：'+str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
