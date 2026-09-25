import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import time
import matplotlib.animation as animation
from matplotlib import style

base_path = Path('F:\\tiptilt')
chosen_folder = base_path / '20230413_092145'
print (chosen_folder.name)

NX = 256
NY = 256
NNX = 128
NNY = 128
flat_file_path = Path('C:\\Users\\IIAP-IPC\\Desktop\\harsh\\TipTiltFLI-master\\cmake-build-debug\\MeanFlat.dat')
dark_file_path = Path('C:\\Users\\IIAP-IPC\\Desktop\\harsh\\TipTiltFLI-master\\cmake-build-debug\\MeanDark.dat')
master_dark = np.mean(np.loadtxt(dark_file_path).reshape(NX, NY).reshape(NNX, NX // NNX, NNY, NY // NNY), axis=(1, 3))
master_flat = np.mean(np.loadtxt(flat_file_path).reshape(NX, NY).reshape(NNX, NX // NNX, NNY, NY // NNY), axis=(1, 3))

def animate_all(fps=3):
    rate = 1000 / fps
    global im, text
    fig, ax1 = plt.subplots(1, 1)
    im = ax1.imshow(np.zeros((NNX, NNY)), cmap='gray', origin='lower')
    text = fig.text(0.5, 0.9, '2', fontsize=12)
    def animate(i):
        global im, text
        #if i < 10:
        #    file_format = 'Y_frame_00000{}_000001_Cur.dat'
        #elif i < 100:
        #    file_format = 'Y_frame_0000{}_000001_Cur.dat'
        #else:
        #    file_format = 'Y_frame_000{}_000001_Cur.dat'
        file_format = 'frame_{}_curr.dat'
        cc = i
        filename = chosen_folder / file_format.format(cc)
        if filename.exists():
            #img = np.mean(np.fromfile(filename, dtype='H').reshape(NX, NY).reshape(NNX, NX // NNX, NNY, NY // NNY), axis=(1, 3))
            img = np.mean(np.loadtxt(filename).reshape(NX, NY).reshape(NNX, NX // NNX, NNY, NY // NNY), axis=(1, 3))
            #img = (img - master_dark) / (master_flat - master_dark)
            im.set_array(img)
            text.set_text('Frame {}'.format(cc))
            im.set_clim(img.min(), img.max())
        return [im]
    ani = animation.FuncAnimation(fig, animate, interval=rate, blit=False)
    plt.show()
    return ani


if __name__ == '__main__':
    im = None
    text = None
    ani = animate_all()