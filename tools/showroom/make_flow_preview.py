"""Make a single car's entrance / idle / selection review movie from frames."""
import argparse
from pathlib import Path
from package_media import resolve_ffmpeg, run, verify_video

parser = argparse.ArgumentParser()
parser.add_argument('--car', required=True, choices=('amr23', 'jesko', 'urus', 'rb19'))
parser.add_argument('--frames', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
args.out.mkdir(parents=True, exist_ok=True)
ffmpeg = resolve_ffmpeg(None)
output = args.out / (args.car + '-flow.mp4')
run(ffmpeg, ['-v', 'warning', '-y', '-framerate', '24', '-start_number', '1',
             '-i', str(args.frames / args.car / '%04d.png'),
             '-vf', "select='between(n,0,119)+between(n,156,227)',setpts=N/24/TB",
             '-frames:v', '192', '-an', '-c:v', 'libx264', '-preset', 'medium',
             '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)],
    args.out / (args.car + '-flow-encode.log'))
print(verify_video(ffmpeg, output, 192, args.out / (args.car + '-flow-decode.log')))
print(output.resolve())
