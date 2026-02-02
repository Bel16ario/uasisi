#TODO: Add a way to scale the plots manually or scale based on the maximum data across all steps. I do not like the scale changing as the data is stepped.

import sys
import h5py
import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.widgets import Slider, Button, CheckButtons 
from matplotlib.gridspec import GridSpec 
from matplotlib.animation import FuncAnimation
from pathlib import Path 
from typing import Dict, List 
import argparse 
from glob import glob 


class H5Viewer:

    COLORS = { #Copied from citruszest nvim theme
        'background': '#121212',
        'foreground': '#BFBFBF',
        'visual': '#404040',
        'cursor': '#383838',
        'black': '#232323',
        'red': '#FF5454',
        'green': '#00CC7A',
        'yellow': '#FFD700',
        'orange': '#FF7431',
        'blue': '#00BFFF',
        'cyan': '#00FFFF',
        'white': '#BFBFBF',
        'bright_green': '#1AFFA3',
        'bright_blue': '#28C9FF',
    }

    def __init__(self, fPath: str):
        self.fPath = Path(fPath)
        if not self.fPath.exists():
            raise RuntimeError("Data file not found: {fPath}")

        self.loadData(h5py.File(self.fPath, 'r'))

        self.step = 0
        self.fractionalStep = 0.0
        self.isPlaying = False
        self.playSpeed = 1
        self.selectedSignals = []
        self.timer = None
        
        self.drawUI()

        self.timer = FuncAnimation(self.figure, self.animate, interval=16, blit=False, repeat=True, cache_frame_data=False)
        self.timer.event_source.stop()


    def loadData(self, file: h5py.File):
        self.time = file['time'][:]
        self.nSteps = len(self.time)
        self.signals = {}
        allSignals = file['signals']

        for signalName in allSignals.keys():
            signalData = {}
            signalGroup = allSignals[signalName]

            signalData['data'] = signalGroup['data'][:]
            signalData['shape'] = signalData['data'].shape

            if 'coords' in signalGroup:
                signalData['coords'] = signalGroup['coords'][:]
                if len(signalData['shape']) == 2:
                    signalData['dataType'] = 'DOB'
                elif len(signalData['shape']) == 3:
                    signalData['dataType'] = 'VEC'
                else:
                    raise RuntimeError("Cannot recognise data type")
            else:
                signalData['dataType'] = 'SCA'

            self.signals[signalName] = signalData

        print(f"Found  {len(self.signals)} signals and {self.nSteps} steps")

    def drawUI(self):

        plt.rcParams['figure.facecolor'] = self.COLORS['background']
        plt.rcParams['axes.facecolor'] = self.COLORS['background']
        plt.rcParams['axes.edgecolor'] = self.COLORS['foreground']
        plt.rcParams['axes.labelcolor'] = self.COLORS['foreground']
        plt.rcParams['text.color'] = self.COLORS['foreground']
        plt.rcParams['xtick.color'] = self.COLORS['foreground']
        plt.rcParams['ytick.color'] = self.COLORS['foreground']
        plt.rcParams['grid.color'] = self.COLORS['visual']
        plt.rcParams['toolbar'] = 'toolbar2'

        self.figure = plt.figure(figsize=(16, 10))
        self.figure.patch.set_facecolor(self.COLORS['background'])
        self.figure.canvas.manager.set_window_title(f'UASISI - {self.fPath.name}')
        self.figure.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)

        grid = GridSpec(5, 3, figure=self.figure, height_ratios=[0.5, 4.5, 0.5, 4.5, 0.8], width_ratios=[5, 5, 2], hspace=0.3, wspace=0.3)
        

        self.ax1 = self.figure.add_subplot(grid[1, 0])
        self.ax2 = self.figure.add_subplot(grid[3, 0])
        self.ax3 = self.figure.add_subplot(grid[1, 1])
        self.ax4 = self.figure.add_subplot(grid[3, 1])
        self.axes = [self.ax1, self.ax2, self.ax3, self.ax4]

        self.controls = self.figure.add_subplot(grid[1, 2])
        self.controls.axis('off')
        
        self.signalList = self.figure.add_subplot(grid[3, 2])
        self.signalList.axis('off')

        self.slider = self.figure.add_subplot(grid[4, :])
        self.timeSlider = Slider(self.slider, 'Step', 0, self.nSteps - 1, valinit=0, valstep=1, valfmt='%d', color=self.COLORS['cyan'], track_color=self.COLORS['visual'])
        self.timeSlider.label.set_color(self.COLORS['foreground'])
        self.timeSlider.valtext.set_color(self.COLORS['yellow'])
        self.timeSlider.on_changed(self.sliderChange)

        self.setControls()
        self.autoSelectSignals()
        self.update()

    def setControls(self):
        boundingBox = self.controls.get_position()
        hBuffer = boundingBox.width*0.075
        vBuffer = boundingBox.height*0.05
        bWidth = (boundingBox.width - hBuffer)/2
        bHeight = (boundingBox.height - vBuffer)/4
        
        play = plt.axes([boundingBox.x0, boundingBox.y1 - bHeight, boundingBox.width, bHeight])
        prev = plt.axes([boundingBox.x0, boundingBox.y1 - (2*bHeight + vBuffer), bWidth, bHeight])
        next = plt.axes([boundingBox.x0 + bWidth + hBuffer, boundingBox.y1 - (2*bHeight + vBuffer), bWidth, bHeight])
        slow = plt.axes([boundingBox.x0, boundingBox.y1 - (3*bHeight + 2*vBuffer), bWidth, bHeight])
        fast = plt.axes([boundingBox.x0 + bWidth + hBuffer, boundingBox.y1 - (3*bHeight + 2*vBuffer), bWidth, bHeight])
        reset = plt.axes([boundingBox.x0, boundingBox.y1 - (4*bHeight + 3*vBuffer), bWidth, bHeight])
        export = plt.axes([boundingBox.x0 + bWidth + hBuffer, boundingBox.y1 - (4*bHeight + 3*vBuffer), bWidth, bHeight])

        self.playButton = Button(play, '▶/❚❚', color=self.COLORS['visual'], hovercolor=self.COLORS['orange'])
        self.playButton.label.set_color(self.COLORS['yellow'])
        self.playButton.on_clicked(self.togglePlay)

        self.prevButton = Button(prev, '◀', color=self.COLORS['visual'], hovercolor=self.COLORS['orange'])
        self.prevButton.label.set_color(self.COLORS['yellow'])
        self.prevButton.on_clicked(lambda x: self.stepThrough(-1))

        self.nextButton = Button(next, '▶', color=self.COLORS['visual'], hovercolor=self.COLORS['orange'])
        self.nextButton.label.set_color(self.COLORS['yellow'])
        self.nextButton.on_clicked(lambda x: self.stepThrough(1))

        self.slowButton = Button(slow, '-', color=self.COLORS['visual'], hovercolor=self.COLORS['orange'])
        self.slowButton.label.set_color(self.COLORS['yellow'])
        self.slowButton.on_clicked(lambda x: self.changeSpeed(-0.25))

        self.fastButton = Button(fast, '+', color=self.COLORS['visual'], hovercolor=self.COLORS['orange'])
        self.fastButton.label.set_color(self.COLORS['yellow'])
        self.fastButton.on_clicked(lambda x: self.changeSpeed(0.25))

        self.resetButton = Button(reset, 'RESET', color=self.COLORS['red'], hovercolor=self.COLORS['orange'])
        self.resetButton.label.set_color(self.COLORS['yellow'])
        self.resetButton.on_clicked(self.reset)

        self.exportButton = Button(export, 'EXPORT', color=self.COLORS['red'], hovercolor=self.COLORS['orange'])
        self.exportButton.label.set_color(self.COLORS['yellow'])
        self.exportButton.on_clicked(self.exportFrame)

        signalNames = list(self.signals.keys())
        visibleSignals = signalNames[:len(signalNames)]
      
        self.enableSignalButtons = CheckButtons(
            ax=self.signalList,
            labels=visibleSignals,
            actives=[name in self.selectedSignals for name in visibleSignals],
            label_props={'color': [self.COLORS['foreground']] * len(visibleSignals)},
            frame_props={'edgecolor': [self.COLORS['visual']] * len(visibleSignals), 
                         'facecolor': [self.COLORS['background']] * len(visibleSignals)},
            check_props={'facecolor': [self.COLORS['yellow']] * len(visibleSignals)}
        )
        self.enableSignalButtons.on_clicked(self.toggleSignal)
        self.updateStatus()

    def autoSelectSignals(self):
        self.selectedSignals = []

    def toggleSignal(self, label):
        if label in self.selectedSignals:
            self.selectedSignals.remove(label)
        else:
            if len(self.selectedSignals) < 4:
                self.selectedSignals.append(label)
            else: 
                return
                
        self.update()

    def sliderChange(self, val):
        self.step = int(val)
        self.fractionalStep = float(self.step)
        self.update()

    def togglePlay(self, event):
        self.isPlaying = not self.isPlaying
        if self.isPlaying:
            self.playButton.label.set_text('❚❚')
            self.play()
        else:
            self.playButton.label.set_text('▶')
            self.stop()

    def animate(self, frame):
        if not self.isPlaying:
            return
        self.fractionalStep = (self.fractionalStep + self.playSpeed) % self.nSteps
        self.step = int(self.fractionalStep)
        self.timeSlider.set_val(self.step)

    def play(self):
        if self.timer is not None:
            self.timer.event_source.start()
    
    def stop(self):
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.event_source.stop()

    def stepThrough(self, n):
        self.step = np.clip(self.step + n, 0, self.nSteps - 1)
        self.timeSlider.set_val(self.step)

    def changeSpeed(self, n):
        self.playSpeed += n            
        self.updateStatus()

    def reset(self, event):
        self.step = 0 
        self.fractionalStep = 0.0
        self.timeSlider.set_val(0)
        if self.isPlaying:
            self.togglePlay(None)

    def updateStatus(self):
        title = f'UASISI - {self.fPath.stem}\n'
        title += f"Step: {self.step}/{self.nSteps-1}  |  "
        title += f"Time: {self.time[self.step]:.4f} s  |  "
        title += f"Speed: {self.playSpeed}x"
        self.figure.suptitle(title, fontsize=14, fontweight='bold')

    def update(self):
        for ax in self.axes:
            ax.clear()
            ax.set_facecolor(self.COLORS['background'])

        for idx, signalName in enumerate(self.selectedSignals[:4]):
            if idx >= len(self.axes):
                break
            ax = self.axes[idx]
            signal = self.signals[signalName]

            if signal['dataType'] == 'SCA':
                ax.plot(self.time, signal['data'], color=self.COLORS['bright_green'], alpha = 1, linewidth = 1)
                ax.axvline(self.time[self.step], color=self.COLORS['red'], linestyle='--', alpha=1)
                ax.text(0.98, 0.98, f"{signal['data'][self.step]:.4f}", transform=ax.transAxes, ha='right', va='top', fontsize=10, color=self.COLORS['foreground'], bbox=dict(boxstyle='round', facecolor=self.COLORS['visual'], edgecolor = self.COLORS['cyan'], alpha=0.9))
                ax.set_xlabel('t [s]', color=self.COLORS['foreground'])
                ax.set_ylabel(signalName, color=self.COLORS['foreground'])
                ax.grid(True, alpha=0.3, color=self.COLORS['visual'])
            elif signal['dataType'] == 'DOB':
                coords = signal['coords']
                data = signal['data'][self.step, :]
                ax.plot(coords, data, color=self.COLORS['bright_green'], alpha = 1, linewidth = 1)
                ax.set_xlabel('z [m]', color=self.COLORS['foreground'])
                ax.set_ylabel(signalName, color=self.COLORS['foreground'])
                ax.grid(True, alpha=0.3, color=self.COLORS['visual'])
            elif signal['dataType'] == 'VEC':
                coords = signal['coords']
                data = signal['data'][self.step, :, :]
                nComponents = data.shape[1]
                colors = [self.COLORS['blue'], self.COLORS['green'], self.COLORS['yellow'], self.COLORS['orange'], self.COLORS['cyan']]
                for comp in range(nComponents):
                    color = colors[comp % len(colors)]
                    ax.plot(coords, data[:, comp], color=color, label=f'Component {comp}', linewidth=1, alpha = 1)
                ax.set_xlabel('z [m]', color=self.COLORS['foreground'])
                ax.set_ylabel(signalName, color=self.COLORS['foreground'])
                legend = ax.legend(fontsize=8, facecolor=self.COLORS['background'], edgecolor=self.COLORS['visual'])
                plt.setp(legend.get_texts(), color=self.COLORS['foreground'])
                ax.grid(True, alpha=0.3, color=self.COLORS['visual'])

            ax.set_title(f'{signalName} (t={self.time[self.step]:.4f}s)', fontsize = 10, color=self.COLORS['bright_blue'])

            for spine in ax.spines.values():
                spine.set_edgecolor(self.COLORS['visual'])

        self.updateStatus()
        self.figure.canvas.draw_idle()

    def exportFrame(self, event):
        dir = self.fPath.parent / 'exports'
        dir.mkdir(exist_ok=True)
        
        imgFile = dir / f'{self.fPath.stem}_step{self.step:04d}.png'
        self.figure.savefig(imgFile, dpi=150, bbox_inches='tight')
        print(f"Exported: {imgFile}")

        dataFile = dir / f'{self.fPath.stem}_step{self.step:04d}.npz'
        exportData = {'time': self.time[self.step]}
        
        for signalName, signal in self.signals.items():
            if signal['dataType'] == 'SCA':
                exportData[signalName] = signal['data'][self.step]
            else:
                exportData[signalName] = signal['data'][self.step]
                if 'coords' in signal:
                    exportData[f'{signalName}_coords'] = signal['coords']
        
        np.savez(dataFile, **exportData)
        print(f"Exported: {dataFile}")

    def show(self):
        plt.show()

def main():
    parser = argparse.ArgumentParser(
        description='UASISI HDF5 Viewer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
  python viewer.py <FILENAME>
        """
    )
    
    parser.add_argument('file', nargs='?', help='HDF5 file to view')
    args = parser.parse_args()
    
    if not args.file:
        raise RuntimeError("No target file specified")

    viewer = H5Viewer(args.file)
    viewer.show()
    return 0 

if __name__ == '__main__':
    sys.exit(main())
