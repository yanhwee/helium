import subprocess
from subprocess import Popen
from google.protobuf import text_format

def gztopic(topic, msg_type):
    with Popen(['gz', 'topic', '-e', topic], stdout=subprocess.PIPE) as p:
        lines = []
        for x in iter(p.stdout.readline, b'\n'):
            assert(x != b'')
            lines.append(x)
        string = b''.join(lines).decode('utf-8')
        msg = text_format.Merge(string, msg_type())
        return msg