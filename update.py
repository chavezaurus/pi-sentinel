import subprocess

def sync_and_restart(remote_host):
    remote_user="pi"
    remote_dir="/home/pi/pi-sentinel"
    other_dir="/home/pi/pi-admtools"

    files = [
        'pi-sentinel.cpp',
        'pi-sentinel.hpp',
        'sentinel.py',
        './public/index.js',
        './public/index.css'
    ]
    # Build rsync command for dry-run (check)
    dest = f"{remote_user}@{remote_host}:{remote_dir}/"
    rsync_cmd = [
        'rsync', '-avcin', '--itemize-changes', '--relative'
    ] + files + [dest]
    # Dry run to check for changes
    result = subprocess.run(rsync_cmd, capture_output=True, text=True)
    print(result.stdout)
    changed = any(line.startswith(('<f', '*')) for line in result.stdout.splitlines())
    if changed:
        print("Files differ; syncing and restarting remote session...")
        subprocess.run([
            'ssh', f'{remote_user}@{remote_host}',
            f"cd '{remote_dir}' && tmux kill-session -t sentinel"
        ])
        # Actual sync
        rsync_cmd[1] = '-avc'
        subprocess.run(rsync_cmd)
        dest = f"{remote_user}@{remote_host}:{other_dir}/"
        rsync_cmd = [ 'rsync', '-avc', 'start_session.sh', dest]
        subprocess.run([
            'ssh', f'{remote_user}@{remote_host}',
            f"cd '{remote_dir}' && make && cd '{other_dir}' && ./start_session.sh"
        ])
        return False
    else:
        print("All files are identical.")
        return True

if __name__ == "__main__":
    import sys

    for arg in sys.argv[1:]:
        sync_and_restart(arg)
