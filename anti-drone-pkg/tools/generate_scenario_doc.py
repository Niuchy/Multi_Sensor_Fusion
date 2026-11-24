"""生成场景设计说明 Word 文档。"""

from docx import Document
from docx.shared import Pt, Cm, RGBColor, Inches
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml.ns import qn
from docx.oxml import OxmlElement


def set_font(run, name='宋体', size=10.5, bold=False, color=None):
    run.font.name = name
    run.font.size = Pt(size)
    run._element.rPr.rFonts.set(qn('w:eastAsia'), name)
    if bold:
        run.bold = True
    if color:
        run.font.color.rgb = color


def heading(doc, text, level=1):
    h = doc.add_heading(text, level=level)
    for run in h.runs:
        set_font(run, '黑体', {0: 22, 1: 16, 2: 13, 3: 11}.get(level, 10))


def para(doc, text, bold=False, indent=0, size=10.5):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(3)
    if indent:
        p.paragraph_format.left_indent = Cm(indent)
    run = p.add_run(text)
    set_font(run, '宋体', size, bold)
    return p


def mono(doc, text, size=8):
    """等宽字体段落（用于示意图和代码）"""
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(1)
    p.paragraph_format.space_after = Pt(1)
    p.paragraph_format.left_indent = Cm(0.5)
    run = p.add_run(text)
    set_font(run, 'Consolas', size, color=RGBColor(0x44, 0x44, 0x44))
    return p


def table(doc, headers, rows, col_widths=None):
    t = doc.add_table(rows=1 + len(rows), cols=len(headers))
    t.style = 'Light Grid Accent 1'
    t.alignment = 1  # center
    for i, h in enumerate(headers):
        c = t.rows[0].cells[i]
        c.text = ''
        run = c.paragraphs[0].add_run(h)
        set_font(run, '黑体', 9, bold=True)
    for ri, row in enumerate(rows):
        for ci, val in enumerate(row):
            c = t.rows[ri + 1].cells[ci]
            c.text = ''
            run = c.paragraphs[0].add_run(str(val))
            set_font(run, '宋体', 9)
    if col_widths:
        for i, w in enumerate(col_widths):
            for row_obj in t.rows:
                row_obj.cells[i].width = Cm(w)
    doc.add_paragraph()
    return t


def hr(doc):
    doc.add_paragraph('─' * 60)


def build():
    doc = Document()
    sec = doc.sections[0]
    sec.page_width = Cm(21)
    sec.page_height = Cm(29.7)
    sec.left_margin = Cm(2.5)
    sec.right_margin = Cm(2.5)

    # ================================================================
    # 封面
    # ================================================================
    for _ in range(4):
        doc.add_paragraph()
    heading(doc, '反无人机数据生成系统', level=0)
    heading(doc, '场景设计说明', level=0)
    doc.add_paragraph()
    para(doc, '版本：V1.0', size=11)
    para(doc, '日期：2026年6月', size=11)

    doc.add_page_break()

    # ================================================================
    # 目录页（手动）
    # ================================================================
    heading(doc, '目录', 1)
    toc = [
        '1. 概述',
        '2. 场景 1：单目标基础场景',
        '3. 场景 2：多目标复杂机动',
        '4. 场景 3：复杂交战场景',
        '5. 场景 4：饱和式突击（蜂群）',
        '6. 场景 5：侦察-干扰-打击分层协同（蜂群）',
        '7. 场景 6：区域游荡与召唤打击（蜂群）',
        '8. 场景 7：分布式狼群协同搜索（蜂群）',
        '9. 场景使用建议',
        '10. 自定义场景指南',
    ]
    for t in toc:
        para(doc, t, size=11)

    doc.add_page_break()

    # ================================================================
    # 1. 概述
    # ================================================================
    heading(doc, '1. 概述', 1)
    para(doc, '系统包含 7 个预置场景，覆盖从单目标基础验证到 15 架蜂群复杂战术的全谱系。')
    para(doc, '坐标系：ENU（东-北-天），原点在传感器阵地中心 (0, 0, 0)。')
    table(doc,
          ['场景', '时长', '目标数', '类型', '核心战术'],
          [['单目标基础', '90s', '1', '基础', '四旋翼直线接近'],
           ['多目标复杂机动', '120s', '3', '协同', '固定翼+四旋翼+悬停'],
           ['复杂交战', '150s', '3', '实战', '干扰+恶劣天气+低截获'],
           ['饱和突击', '120s', '15', '蜂群', '5方向×3高度层同时突入'],
           ['分层协同', '150s', '15', '蜂群', '侦察-干扰-打击三层杀伤链'],
           ['游荡召唤', '130s', '15', '蜂群', '巡飞弹盘旋→发现→集中攻击'],
           ['狼群搜索', '150s', '15', '蜂群', '5×3阵列协同推进']])

    # ================================================================
    # 2. 场景 1
    # ================================================================
    heading(doc, '2. 场景 1：单目标基础场景', 1)
    para(doc, '文件：config/scenarios/single_target.yaml')
    para(doc, '时长：90 秒 | 目标数：1 | 用途：基础算法验证、单传感器调试')

    heading(doc, '2.1 目标', 2)
    table(doc, ['ID', '标签', '类型', 'RCS', '辐射源'],
          [['1', 'DJI-Mavic3', '四旋翼', '0.01 m²', '2.4GHz OFDM 0.1W 常开']])

    heading(doc, '2.2 轨迹', 2)
    para(doc, '匀速直线接近，逐渐降低高度：')
    mono(doc, '(2000, 500, 80) @ 0s     ← 2km外，高度80m')
    mono(doc, '(1500, 300, 80) @ 30s    ← 保持高度，直线接近')
    mono(doc, '( 800, 100, 60) @ 60s    ← 降低高度')
    mono(doc, '( 200,   0, 40) @ 90s    ← 抵近阵地')

    heading(doc, '2.3 各传感器探测预期', 2)
    table(doc, ['传感器', '探测情况'],
          [['雷达', '全程可探测（5000m范围内），后期SNR高Pd≈100%'],
           ['光电', '目标进入窄视场后才能锁定，约30~40s后雷达cue引导'],
           ['电侦', '目标全程辐射，截获概率80%，间歇检测'],
           ['声探测', '目标进入500m范围后开始检测（约70s后）']])

    # ================================================================
    # 3. 场景 2
    # ================================================================
    heading(doc, '3. 场景 2：多目标复杂机动', 1)
    para(doc, '文件：config/scenarios/multi_target.yaml')
    para(doc, '时长：120 秒 | 目标数：3 | 用途：多目标融合算法验证')

    heading(doc, '3.1 目标', 2)
    table(doc, ['ID', '标签', '类型', 'RCS', '辐射源', '机动'],
          [['1', '固定翼-UAV-A', '固定翼', '0.5 m²', '5.8GHz BPSK 1W 开机', '高速弧形接近'],
           ['2', '四旋翼-UAV-B', '四旋翼', '0.02 m²', '2.45GHz OFDM 0.2W 开机', '蛇形S机动'],
           ['3', '侦察-UAV-C', '四旋翼', '0.008 m²', '电磁静默！', '悬停60s+隐蔽']])

    heading(doc, '3.2 目标1 轨迹（固定翼）', 2)
    mono(doc, '(3000, -500, 200) @ 0s    右前方高空')
    mono(doc, '(2000,    0, 150) @ 40s   转向阵地中央')
    mono(doc, '(1000,  500, 100) @ 80s   转到左侧')
    mono(doc, '( 200,  200,  50) @ 120s  抵近阵地')

    heading(doc, '3.3 目标2 轨迹（四旋翼蛇形）', 2)
    mono(doc, '(500, 800, 60) → (600, 700, 60) → (400, 500, 80) → (300, 600, 80) → (200, 400, 70) → (100, 300, 60)')
    para(doc, 'S形蛇形机动，高度60~80m间变化，模拟侦察规避。')

    heading(doc, '3.4 目标3 轨迹（悬停侦察）', 2)
    mono(doc, '(100, 100, 100) @ 0s    阵地附近')
    mono(doc, '(100, 100, 100) @ 60s   悬停60秒！')
    mono(doc, '( 50, 100, 120) @ 120s  缓慢爬升移动')
    para(doc, '长时间悬停 + 电磁静默 + 极小RCS(0.008m²) = 最难探测目标。')

    heading(doc, '3.5 融合难点', 2)
    table(doc, ['目标', '雷达', '光电', '电侦', '声探测'],
          [['固定翼-A', '✅ 大RCS', '✅ 进入视场后', '✅ 持续辐射', '后期'],
           ['四旋翼-B', '⚠️ 小RCS', '⚠️ 视场窄', '✅ 间歇', '较近时'],
           ['侦察-C', '❌ 极低RCS', '⚠️ 唯一手段', '❌ 静默', '✅ 最近']])

    # ================================================================
    # 4. 场景 3
    # ================================================================
    heading(doc, '4. 场景 3：复杂交战场景', 1)
    para(doc, '文件：config/scenarios/complex_engagement.yaml')
    para(doc, '时长：150 秒 | 目标数：3 | 用途：高保真实战环境验证')

    heading(doc, '4.1 特殊环境条件', 2)
    table(doc, ['条件', '设置', '影响'],
          [['雷达干扰', '噪声干扰 -30dBm', 'SNR降低，Pd下降，测角精度恶化'],
           ['光电天气', '能见度5km + 中等湍流', '检测概率降低至约50%'],
           ['电侦灵敏度', '截获概率降至60%', '检测大幅减少']])

    heading(doc, '4.2 目标', 2)
    table(doc, ['ID', '标签', '类型', 'RCS', '辐射源', '战术角色'],
          [['1', '攻击-UAV-1', '固定翼', '0.3 m²', '6GHz QPSK 2W', '主攻力量'],
           ['2', '侦察-UAV-2', '四旋翼', '0.005 m²', '关机！', '隐蔽侦察'],
           ['3', '自杀-UAV-3', '固定翼', '0.1 m²', '5.8GHz CW 0.5W', '自杀攻击']])

    heading(doc, '4.3 目标3 轨迹（自杀无人机）', 2)
    mono(doc, '(3000, 2000, 400) @ 0s    高空远处')
    mono(doc, '(2000, 1500, 300) @ 30s   降低高度')
    mono(doc, '( 800,  600, 200) @ 70s   加速接近')
    mono(doc, '(   0,    0,  50) @ 100s  撞击传感器阵地！')
    para(doc, '末段直冲（0,0）雷达部署点，模拟自杀式攻击。100s后目标3停在原点。')

    heading(doc, '4.4 融合难点', 2)
    para(doc, '在干扰+恶劣天气+低截获概率的复合环境下，融合算法需要从零散、低质量的数据中重建完整态势。')

    # ================================================================
    # 5. 场景 4：饱和式突击
    # ================================================================
    heading(doc, '5. 场景 4：饱和式突击（蜂群）', 1)
    para(doc, '文件：config/scenarios/swarm_saturation.yaml')
    para(doc, '时长：120 秒 | 目标数：15 | 用途：目标容量压力测试')

    heading(doc, '5.1 战术设计', 2)
    para(doc, '15架固定翼无人机，5方向×3高度层，无领机，牺牲数量保证突防。')
    mono(doc, '     ← -80° (3架)            高空300m  ○○○○○')
    mono(doc, '   ← -40° (3架)              中空150m  ○○○○○')
    mono(doc, ' ←   0° (3架) 正面          低空 50m  ○○○○○')
    mono(doc, '   ← +40° (3架)')
    mono(doc, '     ← +80° (3架)            集中到达时间：80-110s')

    table(doc, ['要素', '设计'],
          [['突入方向', '5个方向，覆盖160°正面扇区（-80°~+80°）'],
           ['高度分层', '3层（50m / 150m / 300m），每方向每层1架'],
           ['机型', '全部固定翼，高速（40-60 m/s）'],
           ['RCS', '0.05~0.3 m²，不追求隐身，靠数量突防'],
           ['辐射管控', '2/3开机，1/3电磁静默'],
           ['到达时序', '集中在最后30秒（80~110s）同时抵达'],
           ['组织方式', '无领机，分布式自主突入']])

    # ================================================================
    # 6. 场景 5：分层协同
    # ================================================================
    heading(doc, '6. 场景 5：侦察-干扰-打击分层协同（蜂群）', 1)
    para(doc, '文件：config/scenarios/swarm_layered.yaml')
    para(doc, '时长：150 秒 | 目标数：15 | 用途：协同杀伤链验证')

    heading(doc, '6.1 三层杀伤链', 2)
    mono(doc, '第一层 侦察机×5：0s前出 ────── 120s抵达前沿')
    mono(doc, '    高空300-400m, 隐身RCS=0.01, 一半间歇辐射')
    mono(doc, '    任务：发现-识别-定位目标')
    mono(doc, '')
    mono(doc, '第二层 电战机×4：10-30s滞后 ── 抵达')
    mono(doc, '    中空150-250m, 大RCS=0.5(外露天线), 持续大功率干扰')
    mono(doc, '    任务：压制防御系统，掩护攻击机')
    mono(doc, '')
    mono(doc, '第三层 攻击机×6：50-70s滞后 ── 130s冲刺')
    mono(doc, '    低空50-200m, 依赖侦察数据链引导')

    table(doc, ['层级', '数量', '角色', '高度', 'RCS', '辐射特征'],
          [['侦察层', '5', 'ISR', '300-400m', '0.01 (隐身)', '间歇'],
           ['电战层', '4', 'EW', '150-250m', '0.5 (大)', '持续干扰'],
           ['打击层', '6', 'Strike', '50-200m', '0.1-0.3', '正常']])

    # ================================================================
    # 7. 场景 6：游荡召唤
    # ================================================================
    heading(doc, '7. 场景 6：区域游荡与召唤打击（蜂群）', 1)
    para(doc, '文件：config/scenarios/swarm_loitering.yaml')
    para(doc, '时长：130 秒 | 目标数：15 | 用途：时敏目标响应测试')

    heading(doc, '7.1 两阶段作战', 2)
    para(doc, '阶段 1（0-70s）：分散游荡搜索', bold=True)
    para(doc, '15架巡飞弹在5×3网格（3km×3km区域）内各自盘旋，半径200-500m，高度80-150m。全电磁静默，被动搜索目标。')
    para(doc, '阶段 2+3（70-130s）：集中攻击', bold=True)
    para(doc, '发现目标后所有巡飞弹从各方汇聚到同一点（100,0）。攻击阶段辐射源开启（active_after~70s，末制导雷达/数据链），高度降至30m。')

    heading(doc, '7.2 探测难点', 2)
    table(doc, ['阶段', '雷达', '电侦', '声探测'],
          [['游荡(0-70s)', '⚠️ 极小RCS(0.005-0.02)', '❌ 电磁静默', '❌ 距离远'],
           ['攻击(70-130s)', '✅ 距离变近', '✅ 末制导开机', '✅ 进入范围']])

    # ================================================================
    # 8. 场景 7：狼群搜索
    # ================================================================
    heading(doc, '8. 场景 7：分布式狼群协同搜索（蜂群）', 1)
    para(doc, '文件：config/scenarios/swarm_wolfpack.yaml')
    para(doc, '时长：150 秒 | 目标数：15 | 用途：广域搜索覆盖验证')

    heading(doc, '8.1 阵列配置', 2)
    mono(doc, '后排 5架 @ ~300m  — 广域监视+通信中继')
    mono(doc, '中排 5架 @ ~150m  — 中距离搜索')
    mono(doc, '前排 5架 @  ~60m  — 前沿抵近侦察')
    mono(doc, '间距 800-1000m，整体从X=-3000推进到X=+3000')
    mono(doc, '每架走之字形（S形），奇偶列反相摆动，增大覆盖')

    heading(doc, '8.2 辐射管控', 2)
    para(doc, '2/3无人机开启辐射源，1/3保持静默。开/关交替轮换，避免全部暴露。')

    # ================================================================
    # 9. 使用建议
    # ================================================================
    heading(doc, '9. 场景使用建议', 1)
    table(doc, ['用途', '推荐场景'],
          [['单传感器调试', 'single_target'],
           ['融合算法基础验证', 'multi_target'],
           ['抗干扰测试', 'complex_engagement'],
           ['目标容量压力测试', 'swarm_saturation'],
           ['协同杀伤链验证', 'swarm_layered'],
           ['时敏目标响应测试', 'swarm_loitering'],
           ['广域搜索覆盖验证', 'swarm_wolfpack']])

    # ================================================================
    # 10. 自定义场景
    # ================================================================
    heading(doc, '10. 自定义场景指南', 1)
    para(doc, '方法一：手写 YAML 文件，参考 single_target.yaml 格式。')
    para(doc, '方法二：修改 tools/generate_scenarios.py 中的战术参数，运行生成：')
    mono(doc, 'python tools/generate_scenarios.py')
    doc.add_paragraph()
    heading(doc, '关键参数速查', 2)
    mono(doc, 'targets:')
    mono(doc, '  - id: 1')
    mono(doc, '    label: "名称"')
    mono(doc, '    type: quadcopter        # quadcopter / fixed_wing')
    mono(doc, '    rcs: 0.01               # 雷达散射截面(m²)')
    mono(doc, '    emitter:')
    mono(doc, '      active: true          # 辐射源开关')
    mono(doc, '      freq: 2.4e+09         # 载频(Hz)')
    mono(doc, '      modulation: OFDM      # 调制类型')
    mono(doc, '      active_after: 70.0    # 此时间后开机(秒,可选)')
    mono(doc, 'simulation:')
    mono(doc, '  duration: 120             # 仿真时长(秒)')
    mono(doc, '  dt: 0.05                  # 步长(保持0.05)')
    mono(doc, '  seed: 12345               # 随机种子')

    # 保存
    output = 'docs/场景设计说明.docx'
    doc.save(output)
    print(f'已生成: {output}')


if __name__ == '__main__':
    build()
