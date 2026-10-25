"""Minimal PICO HUD: operator guidance from actual control/stop feedback.

This is an observer. Key labels are instructions, never clickable commands.
The six reference states retain separate startup/preparation guidance.
"""
from functools import lru_cache
import os
from pathlib import Path


def hud_spec(view):
    mode, reason = view.get('mode', 'checking'), view.get('reason', '')
    if view.get('fault_active') and view.get('first_fault'):
        reason = view['first_fault']['code']
    confirmed = view.get('stop_confirmed') is True
    applied = view.get('grasp_applied')
    # Older observers cannot prove the applied grasp from a requested value.
    grip = '右手虚握' if applied == 'right_axial' else '刚握' if applied == 'rigid' else '握持待确认'
    spec = dict(state='paused', status='待命', grip=grip, tone='warning',
                title='', detail='', action='Y 停止', context='模式待确认',mode_hint='等待有效控制状态')
    independent=view.get('teleop_mode')=='independent_arms'
    if independent:spec.update(context='普通双臂',mode_hint='左右手柄各控一臂',grip='逐臂离合')
    elif view.get('teleop_mode')=='bimanual_tool':
        spec.update(context='扫雪耦合',mode_hint='双手柄共同控制工具')
        if view.get('tool_grasp_confirmation')=='operator':spec['grip']='人工确认 · 刚握'
        if view.get('tool_variant')=='short_rod':spec.update(context='短棍耦合',mode_hint='双手柄共同控制短棍')
    if mode in ('DISPLAY_STALE','DISPLAY_WAITING'):
        spec.update(context='模式待确认',mode_hint='等待有效控制状态')
    def notice(title, detail):
        spec.update(title=title, detail=detail)
    input_fault = any(x in reason for x in ('INPUT_TIMEOUT','PAIR_INVALID','POSE_INVALID','TRACKING','CONNECTION_CHANGED','POSE_ORIGIN','TIME_REVERSED','HEADSET'))
    if mode == 'DISPLAY_STALE':
        spec.update(state='lost', status='状态未知', tone='danger',grip='握持未知')
        notice('状态反馈已中断', '停止状态未确认，请查看电脑操作台')
    elif mode == 'locked':
        spec.update(state='lost' if input_fault else 'stopped',
                    status='已停止 · 锁定' if confirmed else '停止未确认', tone='danger')
        if 'VR_HEADSET' in reason:
            notice('头显朝向不可用', '恢复VR追踪并平视操作前方；松握后按 B' if confirmed else
                   '保持松握，等待停稳确认；检查头显追踪')
        elif 'PICO_ACTUATOR_FEEDBACK_TIMEOUT' in reason:
            notice('未到目标位置',
                   '已停稳；查看电脑端关节误差，松握按 B 恢复待命' if confirmed else
                   '到位等待超时；保持松握，等待停稳确认')
        elif input_fault:
            hands = view.get('controller_states', {})
            invalid = [label for side,label in (('left','左'),('right','右'))
                       if not view.get('tracking_valid',{}).get(side,hands.get(side, {}).get('valid',False))]
            title = ('、'.join(invalid)+'手柄输入失效') if invalid and len(invalid)<2 else '双手输入已中断'
            notice(title, '已确认停稳；恢复追踪、松握后按 B' if confirmed else '等待停稳确认；恢复追踪并松开双侧握')
        else:
            title = '已确认停稳' if confirmed else '等待停稳确认'
            detail = '检查环境，松开双侧握后按 B 恢复待命' if confirmed else '保持松握；未确认前请查看电脑操作台'
            if reason != 'PICO_Y_STOP':
                title = '控制链阻断' if confirmed else '控制异常 · 停止未确认'
                detail = '查看电脑端原因；排除后松握并按 B' if confirmed else detail
            notice(title,detail)
            if 'TOOL_BELOW_GROUND' in reason:
                notice('工具触地策略已阻断' if confirmed else '等待停稳确认',
                    '查看工具长度与场景策略；松握后按 B 恢复' if confirmed else detail)
        if confirmed:spec['action']='B 恢复待命'
        if view.get('stop_failure') and view.get('fault_active'):
            spec['action']='Y 保持停止'
        if view.get('recovery_required') and view.get('fault_active'):
            notice('设备／采样异常 · 锁定','保持松握；查看电脑端首故障与停止协议，需现场排查')
            spec['action']='Y 保持停止'
    elif mode in ('checking','DISPLAY_WAITING'):
        spec.update(status='连接检查',grip='工具')
        notice('等待手柄与关节反馈', '保持双侧握松开，等待连接就绪')
    elif mode == 'preparing':
        spec.update(status='准备中',grip='工具')
        notice('正在前往准备姿态', '继续按住 A；松开 A 即暂停准备')
    elif mode == 'selecting_mode':
        target={'independent_arms':'普通双臂','bimanual_tool':'扫雪耦合'}.get(view.get('mode_target'),'目标')
        spec['status']='切换模式中'
        notice('正在检查'+target+'模式', '保持松握；切换完成后仍待命，不自动运动')
    elif mode in ('confirming_tool','engaging'):
        spec['status']='工具检查中' if mode=='confirming_tool' else '正在接合'
        notice('正在确认工具' if mode=='confirming_tool' else '正在建立相对跟随基准',
                '保持双侧握松开，等待检查完成' if mode=='confirming_tool' else '保持双侧握；松开任一侧可暂停')
        if mode=='engaging' and view.get('direction_reference')=='headset_on_engage':
            notice('正在锁定操作方向', '看向操作正前方并保持侧握；松握可取消')
    elif mode == 'switching':
        requested = view.get('grasp_requested')
        target = '右手虚握' if requested=='right_axial' else '刚握'
        spec.update(state='switching',status='换握中')
        notice('正在确认'+target if view.get('motion_stopped') is True else '等待机器人停稳',
                '尚未进入'+target+'，等待握持确认')
    elif mode == 'following':
        if view.get('input_fresh') is not True or view.get('raw_input_valid') is not True:
            spec.update(state='lost',status='输入失效',tone='danger')
            notice('手柄输入不可用', '松开双侧握，等待控制器确认停止')
        else:
            spec.update(state='virtual' if applied=='right_axial' else 'following',
                        status='跟随中',tone='good')
            limit=view.get('motion_limit')
            if limit:
                budget=limit.get('search_limited_by_budget')
                reason=limit.get('first_rejection','')
                cause=('控制更新暂时延迟' if budget else
                       '手臂接近本体' if reason.startswith('SELF_COLLISION:') else
                       '工具接近头部或本体' if '_CLEARANCE:' in reason else '握持处接近几何边界')
                spec.update(status='跟随受限',tone='warning')
                notice(cause,'保持握住，稍向反方向移动；松握可暂停')
    elif mode == 'tool_standby':
        spec['status']='已暂停' if confirmed else '正在暂停'
        notice('松开双侧握，再同时握住' if confirmed else '等待机器人停稳',
                '重新接合后继续跟随' if confirmed else '保持双侧握松开；停稳后再重新接合')
        if independent and confirmed:notice('握住对应侧，继续控制该臂','重新接合以该臂当前位置建立基准；B 切模式')
        if view.get('operator_guide'):
            guide=view['operator_guide'];notice(guide['title'],guide['detail'])
    elif mode == 'standby':
        if view.get('operator_guide'):
            guide=view['operator_guide'];notice(guide['title'],guide['detail'])
        elif reason in ('PREPARE_REQUIRES_EMPTY_AND_RELEASED_SQUEEZE','PREPARE_REQUIRES_STOP_AND_RELEASED_SQUEEZE'):
            check=view.get('last_prepare_check') or {}
            labels={'load_empty':'当前已装载' if check.get('load_state')=='held' else '空手状态未确认',
                    'feedback_fresh':'反馈无效或过期','motion_stopped':'尚未停稳',
                    'input_fresh':'输入过期','left_squeeze_released':'左侧握未松开','right_squeeze_released':'右侧握未松开'}
            failures=[labels[k] for k,v in check.get('checks',{}).items() if v is False and k in labels]
            notice('准备被拒绝；排除原因后重新长按 A', '、'.join(failures[:3]) or '保持松握，查看电脑端检查详情')
        elif reason=='PREPARE_RELEASED':
            notice('准备已暂停', '松开双侧握，停稳后重新长按 A')
        elif view.get('load_state')=='held':
            notice('工具待确认', '保持松握，等待检查；严格流程按 B 确认')
        elif view.get('capabilities',{}).get('prepare') is False:
            notice('现场准备位尚未配置', '点击左摇杆开始装夹，按屏幕步骤操作')
        else:
            spec['grip']='工具未放置'
            notice('长按 A 前往准备姿态', '保持双侧握松开，准备完成后放置工具' if view.get('simulation_demo') else '保持双侧握松开；准备后单独装夹并确认工具')
    elif mode=='gripper_control':
        spec['status']='夹爪开合中'
        guide=view.get('operator_guide')
        if guide:notice(guide['title'],guide['detail'])
        else:notice('夹爪独立开合', '按住对应扳机打开，松开闭合；保持双侧握松开')
    elif mode in ('loading','load_check'):
        spec['status']='装夹中' if view.get('loading_goal')=='held' else '卸载中'
        guide=view.get('operator_guide')
        if guide:notice(guide['title'],guide['detail'])
        else:notice('装卸需要独立确认', '查看电脑端工具与支撑状态，按既有流程操作')
    else:
        notice('控制状态待确认', '保持松握，查看电脑操作台')
    if view.get('idle_input_lost') and mode in ('standby','tool_standby','load_check'):
        spec['status']='等待手柄输入'
        notice('手柄输入暂停，当前步骤已保留','保持松握，恢复输入后按当前步骤继续')
    return spec


@lru_cache(maxsize=1)
def _font_path():
    candidates = [os.getenv('BINDU_HUD_FONT',''),
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/arphic/uming.ttc']
    for candidate in candidates:
        if candidate and Path(candidate).is_file():return candidate
    raise RuntimeError('HUD_CJK_FONT_MISSING: set BINDU_HUD_FONT to a local CJK font')


@lru_cache(maxsize=64)
def hud_image(state, status, grip, tone, title, detail, action, context, mode_hint):
    """Cache PNG-ready artwork; no font fetch, per-frame metrics or scene images."""
    from PIL import Image, ImageDraw, ImageFont
    image = Image.new('RGBA',(1440,330),(0,0,0,0))
    draw = ImageDraw.Draw(image)
    def font(size):return ImageFont.truetype(_font_path(),size)
    ink, muted, edge = '#e5eeea','#a7bbb0','#41574b'
    panel, inset = '#182721','#263a30'
    color = {'good':'#72dab7','warning':'#f0c476','danger':'#ffaaa0'}[tone]
    # Mode and hand mapping stay visible independently of follow/stop status.
    draw.rounded_rectangle((60,196,1380,326),radius=18,fill=panel,outline=edge,width=2)
    mode_color='#9fc7ff' if context=='普通双臂' else '#80debf' if context in ('扫雪耦合','短棍耦合') else muted
    draw.text((88,203),'当前控制模式',font=font(19),fill=muted)
    draw.text((88,227),context,font=font(38),fill=mode_color)
    draw.text((88,281),mode_hint,font=font(24),fill=muted)
    draw.line((460,218,460,304),fill=edge,width=2)
    draw.ellipse((490,232,502,244),fill=color)
    draw.text((520,215),status,font=font(30),fill=ink)
    draw.text((520,274),grip,font=font(26),fill=muted)
    draw.line((1058,218,1058,304),fill=edge,width=2)
    draw.rounded_rectangle((1084,244,1118,278),radius=17,fill=inset,outline=edge,width=2)
    draw.text((1093,244),action[0],font=font(25),fill=ink)
    draw.text((1130,244),action[2:],font=font(25),fill=ink)
    if title:
        draw.rounded_rectangle((175,28,1265,178),radius=22,fill=panel,outline=edge,width=2)
        draw.rounded_rectangle((198,54,204,150),radius=3,fill=color)
        draw.text((230,48),title,font=font(34),fill=ink)
        draw.text((230,105),detail,font=font(27),fill=muted)
    return image


@lru_cache(maxsize=64)
def _hud_png(**spec):
    from io import BytesIO
    buffer=BytesIO()
    hud_image(**spec).save(buffer,format='PNG')
    return buffer.getvalue()


def render_hud(view):
    # In Vuer 0.0.60 HUDPlane ignores src; ImageBackground owns the texture
    # and has the same camera-relative plane placement and transparency.
    from vuer.schemas import ImageBackground
    import base64
    spec = hud_spec(view)
    # A stable URL string prevents the frontend from allocating a new texture
    # whenever an otherwise unchanged HUD accompanies a joint-state update.
    src='data:image/png;base64,'+base64.b64encode(_hud_png(**spec)).decode('ascii')
    return ImageBackground(src=src,geometry='plane',key='teleopStatus',
        distanceToCamera=1.,height=.25,aspect=1440/330,position=[0.,-.28,0.],
        interpolate=True,opacity=1.,material={'depthTest':False,'depthWrite':False,'toneMapped':False},
        userData={'hud_state':spec['state'],'status':spec['status'],
                  'control_mode':spec['context'],'control_hint':spec['mode_hint'],
                  'title':spec['title'],'detail':spec['detail']})
