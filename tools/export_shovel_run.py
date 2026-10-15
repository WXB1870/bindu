#!/usr/bin/env python3
"""Export one scored physics run as full-step data, figures and a timed video."""
import argparse
import atexit
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np


def stats(values, times):
    return dict(peak=float(np.max(values)), rms=float(np.sqrt(np.mean(values**2))),
                end=float(values[-1]), peak_time_s=float(times[np.argmax(values)]))


def export(directory, overlay_video=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory = Path(directory).resolve()
    result = json.loads((directory/'result.json').read_text())
    score = json.loads((directory/'tracking.json').read_text())
    for name, expected in [('samples.npz', result['provenance']['samples_sha256']),
                           ('tracking.npz', score['tracking_samples_sha256'])]:
        if hashlib.sha256((directory/name).read_bytes()).hexdigest() != expected:
            raise ValueError('EXPORT_SAMPLE_IDENTITY:'+name)
    samples = np.load(directory/'samples.npz')['samples']
    samples = samples[(samples[:,0] >= 0) & (samples[:,0] <= result['duration_s']+1e-9)]
    tracking = np.load(directory/'tracking.npz')
    t = tracking['time_s']
    if len(t) != len(samples) or not np.allclose(t, samples[:,0], atol=1e-10, rtol=0):
        raise ValueError('EXPORT_TIME_GRID_MISMATCH')
    position = tracking['position_error_m']*1000
    angle = np.rad2deg(tracking['orientation_error_rad'])
    load = np.column_stack([np.linalg.norm(samples[:,1+3*i:4+3*i],axis=1) for i in range(2)])
    force = np.column_stack([np.linalg.norm(samples[:,13+6*i:16+6*i],axis=1) for i in range(2)])
    moment = np.column_stack([np.linalg.norm(samples[:,16+6*i:19+6*i],axis=1) for i in range(2)])
    axial = samples[:,25]
    values = np.column_stack([t,position,angle,load,force,moment,axial,samples[:,45]])
    columns = ['time_s','position_error_mm','orientation_error_deg','left_load_n','right_load_n',
               'left_internal_force_n','right_internal_force_n','left_internal_moment_nm',
               'right_internal_moment_nm','axial_internal_force_n','right_axial_released']
    np.savetxt(directory/'full_process.csv', values, delimiter=',', header=','.join(columns),
               comments='', fmt='%.10g')
    summary = dict(sample_count=len(t), start_s=float(t[0]), end_s=float(t[-1]),
                   position_error_mm=stats(position,t), orientation_error_deg=stats(angle,t),
                   load_n={s:stats(load[:,i],t) for i,s in enumerate(('left','right'))},
                   internal_force_n={s:stats(force[:,i],t) for i,s in enumerate(('left','right'))},
                   internal_moment_nm={s:stats(moment[:,i],t) for i,s in enumerate(('left','right'))},
                   axial_internal_abs_n=stats(abs(axial),t),
                   experiment_accepted=score['experiment_accepted'],
                   definitions=dict(internal='grasp-mode constrained self-equilibrating projection with grip-spacing moment weight',
                       axial='positive compression / negative tension; one internal-force component',
                       interval='complete task interval; settling and post-task controller-stop tail excluded',
                       units='N and N*m; internal moments are not wrist sensor moments'),
                   source_sha256=dict(samples=result['provenance']['samples_sha256'],tracking=score['tracking_samples_sha256']))
    from matplotlib import font_manager
    from fontTools.ttLib import TTFont
    font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if not font.is_file():
        raise RuntimeError('EXPORT_CJK_FONT_REQUIRED:'+str(font))
    font_dir = tempfile.TemporaryDirectory(prefix='shovel-plot-font-')
    atexit.register(font_dir.cleanup)
    sc_font = Path(font_dir.name)/'noto-sc.otf'
    TTFont(font, fontNumber=2).save(sc_font)
    font_manager.fontManager.addfont(str(sc_font))
    family = font_manager.FontProperties(fname=str(sc_font)).get_name()
    plt.rcParams.update({'font.family':family, 'font.size':13,
                         'axes.spines.top':False,'axes.spines.right':False,'axes.unicode_minus':False,
                         'path.simplify':False,'pdf.fonttype':42})
    blue, green = '#4472C4', '#548235'

    def decorate(axes):
        released = samples[:,45] > .5
        edges = np.diff(np.r_[False,released,False].astype(int))
        for ax in axes:
            for start,end in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
                ax.axvspan(t[start],t[min(end,len(t)-1)],color='#FFC000',alpha=.1,zorder=0)
            ax.grid(color='#DDDDDD',linewidth=.6)
            ax.set_xlim(t[0],t[-1])
            ax.tick_params(labelsize=12)
        axes[-1].set_xlabel('动作时间（s）')

    fig, axes = plt.subplots(2,1,figsize=(12,7.4),sharex=True,layout='constrained')
    axes[0].plot(t,position,color=blue,linewidth=1.15)
    axes[1].plot(t,angle,color=green,linewidth=1.15)
    axes[0].set_ylabel('位置误差（mm）')
    axes[1].set_ylabel('姿态误差（°）')
    axes[0].set_title('物理闭环全程跟踪误差',fontsize=17,pad=12)
    for ax in axes: ax.set_ylim(bottom=0)
    decorate(axes)
    for suffix in ('png','pdf'): fig.savefig(directory/f'tracking_full.{suffix}',dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(4,1,figsize=(12,9.4),sharex=True,layout='constrained')
    for values,ylabel,ax in zip((load,force,moment),('手部总载荷（N）','分解内力幅值（N）','分解内力矩（N·m）'),axes[:3]):
        ax.plot(t,values[:,0],color=blue,label='左手',linewidth=1.15)
        ax.plot(t,values[:,1],color=green,label='右手',linewidth=1.15,linestyle='--')
        ax.set_ylabel(ylabel)
        ax.set_ylim(bottom=0)
        ax.legend(frameon=False,ncol=2,loc='upper right',fontsize=11)
    axes[3].plot(t,axial,color=blue,linewidth=1.15)
    axes[3].axhline(0,color='#777777',linewidth=.7)
    axes[3].set_ylabel('沿杆内力（N）')
    axes[0].set_title('物理闭环全程双臂载荷与内力',fontsize=17,pad=12)
    decorate(axes)
    for suffix in ('png','pdf'): fig.savefig(directory/f'loads_internal_full.{suffix}',dpi=180)
    plt.close(fig)

    if overlay_video:
        recording = json.loads((directory/'video.json').read_text())
        stamps = np.array(recording['frame_simulation_time_s'])
        expected = np.arange(len(stamps))/recording['fps']
        if not np.allclose(stamps,expected,atol=1e-8,rtol=0):
            raise ValueError('VIDEO_FRAME_TIME_GRID')
        if abs(stamps[-1]-t[-1]) > 1/recording['fps']+1e-9:
            raise ValueError('VIDEO_INCOMPLETE_INTERVAL')
        def ass_time(seconds):
            centiseconds=round(seconds*100)
            return f'{centiseconds//360000}:{centiseconds//6000%60:02d}:{centiseconds//100%60:02d}.{centiseconds%100:02d}'
        subtitle = directory/'video_metrics.ass'
        header = ('[Script Info]\nScriptType: v4.00+\nPlayResX: 1280\nPlayResY: 720\n'
                  '[V4+ Styles]\nFormat: Name,Fontname,Fontsize,PrimaryColour,SecondaryColour,OutlineColour,BackColour,Bold,Italic,Underline,StrikeOut,ScaleX,ScaleY,Spacing,Angle,BorderStyle,Outline,Shadow,Alignment,MarginL,MarginR,MarginV,Encoding\n'
                  'Style: Default,Noto Sans CJK SC,26,&H00FFFFFF,&H00FFFFFF,&H00000000,&H80000000,0,0,0,0,100,100,0,0,1,0,0,7,30,26,24,1\n'
                  '[Events]\nFormat: Layer,Start,End,Style,Name,MarginL,MarginR,MarginV,Effect,Text\n')
        events=[]
        for k,stamp in enumerate(stamps):
            j=int(np.argmin(abs(t-stamp)))
            if abs(t[j]-stamp)>1e-8: raise ValueError('VIDEO_DATA_ALIGNMENT')
            text=(f'空铲物理闭环   {stamp:.2f} s'
                  f'\\N位置误差 {position[j]:.2f} mm  /  姿态误差 {angle[j]:.3f}°'
                  f'\\N内力幅值 左 {force[j,0]:.2f} / 右 {force[j,1]:.2f} N'
                  f'\\N内力矩 左 {moment[j,0]:.2f} / 右 {moment[j,1]:.2f} N·m'
                  f'\\N沿杆内力 {axial[j]:+.2f} N（正压紧、负拉伸）')
            events.append(f'Dialogue: 0,{ass_time(k/recording["fps"])},{ass_time((k+1)/recording["fps"])},Default,,0,0,0,,{text}\n')
        subtitle.write_text(header+''.join(events))
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-nostdin','-y',
                        '-i',str(directory/'physics.mp4'),'-vf',
                        f'drawbox=x=18:y=12:w=590:h=177:color=0x1F4E79@0.78:t=fill,ass={subtitle}',
                        '-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p',
                        '-movflags','+faststart',str(directory/'physics_with_metrics.mp4')],check=True)
        summary['video'] = dict(fps=recording['fps'],frame_count=len(stamps),
                               last_frame_time_s=float(stamps[-1]),synchronization='exact recorded frame simulation times; no time shifting')
    summary['exporter_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (directory/'presentation_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--overlay-video',action='store_true',help='Add measured error and internal wrench values at the recorded frame times')
    args=parser.parse_args()
    export(args.directory,args.overlay_video)
