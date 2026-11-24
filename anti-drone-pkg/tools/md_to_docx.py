"""Markdown → DOCX 转换工具。

将场景设计说明文档转换为 Word 格式。
"""

from docx import Document
from docx.shared import Pt, Inches, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement
import re


def set_cell_shading(cell, color):
    """设置单元格背景色"""
    tc = cell._tc
    tcPr = tc.get_or_add_tcPr()
    shading = OxmlElement('w:shd')
    shading.set(qn('w:fill'), color)
    shading.set(qn('w:val'), 'clear')
    tcPr.append(shading)


def add_table(doc, headers, rows, col_widths=None):
    """添加格式化表格"""
    table = doc.add_table(rows=1 + len(rows), cols=len(headers))
    table.style = 'Light Grid Accent 1'
    table.alignment = WD_TABLE_ALIGNMENT.CENTER

    # 表头
    for i, h in enumerate(headers):
        cell = table.rows[0].cells[i]
        cell.text = h
        for p in cell.paragraphs:
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            for run in p.runs:
                run.bold = True
                run.font.size = Pt(9)

    # 数据行
    for r, row in enumerate(rows):
        for c, val in enumerate(row):
            cell = table.rows[r + 1].cells[c]
            cell.text = str(val)
            for p in cell.paragraphs:
                for run in p.runs:
                    run.font.size = Pt(9)

    if col_widths:
        for i, w in enumerate(col_widths):
            for row in table.rows:
                row.cells[i].width = Cm(w)

    doc.add_paragraph()  # spacing
    return table


def add_code_block(doc, text):
    """添加代码块"""
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(1)
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(2)
    run = p.add_run(text)
    run.font.name = 'Consolas'
    run.font.size = Pt(8)
    run.font.color.rgb = RGBColor(0x33, 0x33, 0x33)
    # 灰色背景
    pPr = p._p.get_or_add_pPr()
    shading = OxmlElement('w:shd')
    shading.set(qn('w:fill'), 'F5F5F5')
    shading.set(qn('w:val'), 'clear')
    pPr.append(shading)


def convert(input_md, output_docx):
    doc = Document()

    # 页面设置
    section = doc.sections[0]
    section.page_width = Cm(21)
    section.page_height = Cm(29.7)
    section.left_margin = Cm(2.5)
    section.right_margin = Cm(2.5)

    # 设置默认字体
    style = doc.styles['Normal']
    font = style.font
    font.name = '宋体'
    font.size = Pt(10.5)
    style.element.rPr.rFonts.set(qn('w:eastAsia'), '宋体')

    with open(input_md, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    i = 0
    in_code = False
    code_lines = []
    in_table = False
    table_data = []

    while i < len(lines):
        line = lines[i].rstrip()

        # 代码块
        if line.startswith('```'):
            if in_code:
                add_code_block(doc, '\n'.join(code_lines))
                code_lines = []
                in_code = False
            else:
                in_code = True
            i += 1
            continue

        if in_code:
            code_lines.append(line)
            i += 1
            continue

        # 标题
        if line.startswith('# ') and not line.startswith('## '):
            doc.add_heading(line[2:], level=0)
        elif line.startswith('## '):
            doc.add_heading(line[3:], level=1)
        elif line.startswith('### '):
            doc.add_heading(line[4:], level=2)
        elif line.startswith('#### '):
            doc.add_heading(line[5:], level=3)

        # 表格（检测 | 分隔的行）
        elif '|' in line and '---' not in line:
            cells = [c.strip() for c in line.split('|')[1:-1]]
            if all(c.startswith('-') for c in line.split('|')[1:-1] if c.strip()):
                i += 1
                continue
            if not in_table:
                table_data = []
                in_table = True
            table_data.append(cells)
        elif in_table and (not line.strip() or not '|' in line):
            if len(table_data) > 1:
                headers = table_data[0]
                rows = table_data[1:]
                col_count = len(headers) if headers else len(rows[0]) if rows else 1
                add_table(doc, headers, rows, None)
            table_data = []
            in_table = False
            if line.strip():
                p = doc.add_paragraph(line)
                p.paragraph_format.space_before = Pt(2)
                p.paragraph_format.space_after = Pt(2)

        # 分隔线
        elif line.strip() == '---':
            doc.add_paragraph('─' * 60)
        # 水平线示意图 (ASCII art)
        elif any(c in line for c in '│├└─┌┬┤┴┼') and not line.startswith('#'):
            p = doc.add_paragraph()
            run = p.add_run(line)
            run.font.name = 'Consolas'
            run.font.size = Pt(7.5)
            run.font.color.rgb = RGBColor(0x55, 0x55, 0x55)
            p.paragraph_format.space_before = Pt(1)
            p.paragraph_format.space_after = Pt(1)

        # 普通段落
        elif line.strip():
            # 处理粗体
            p = doc.add_paragraph()
            p.paragraph_format.space_before = Pt(2)
            p.paragraph_format.space_after = Pt(2)

            # 简单处理 **粗体** 和 `代码`
            parts = re.split(r'(\*\*.*?\*\*|`.*?`)', line)
            for part in parts:
                if part.startswith('**') and part.endswith('**'):
                    run = p.add_run(part[2:-2])
                    run.bold = True
                elif part.startswith('`') and part.endswith('`'):
                    run = p.add_run(part[1:-1])
                    run.font.name = 'Consolas'
                    run.font.size = Pt(9)
                else:
                    p.add_run(part)

        i += 1

    # 处理最后一个表格
    if in_table and len(table_data) > 1:
        add_table(doc, table_data[0], table_data[1:], None)

    doc.save(output_docx)
    print(f'已生成: {output_docx}')


if __name__ == '__main__':
    import sys
    if len(sys.argv) >= 3:
        convert(sys.argv[1], sys.argv[2])
    else:
        convert('docs/场景设计说明.md', 'docs/场景设计说明.docx')
