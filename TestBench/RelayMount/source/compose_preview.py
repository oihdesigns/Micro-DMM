"""Create a readable technical preview with approximate board envelopes."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import json

ROOT = Path(__file__).resolve().parent.parent
data = json.loads((ROOT/'validation.json').read_text())
W, H = 1800, 1080
im = Image.new('RGB',(W,H),'#f4f6f8')
d = ImageDraw.Draw(im)
def font(size,bold=False):
    return ImageFont.truetype('C:/Windows/Fonts/segoeuib.ttf' if bold else 'C:/Windows/Fonts/segoeui.ttf',size)
def text(x,y,t,size=24,fill='#233846',bold=False,anchor=None):
    d.text((x,y),t,font=font(size,bold),fill=fill,anchor=anchor)
text(60,37,'RELAY + ARDUINO MOUNT',42,bold=True)
text(62,96,'6.5 x 5.5 in  |  165.1 x 139.7 mm  |  One printable part',25,fill='#526777')
d.line((60,147,1740,147),fill='#ccd5dc',width=2)
text(62,170,'PRINTED BASE',24,bold=True)
text(1020,170,'TOP VIEW / BOARD PLACEMENT',24,bold=True)
render = Image.open(ROOT/'reference'/'plate_render.png').convert('RGB')
render = render.resize((940,827),Image.Resampling.LANCZOS)
im.paste(render,(35,210))

# Top-view mapping: +Y points upward, matching the photograph.
scale = 3.72
ox, oy = 1060, 845
def xy(x,y): return (ox+x*scale,oy-y*scale)
def box(x,y,w,h,fill,outline,width=2,radius=0):
    a=xy(x,y+h); b=xy(x+w,y)
    if radius: d.rounded_rectangle((*a,*b),radius=radius,fill=fill,outline=outline,width=width)
    else: d.rectangle((*a,*b),fill=fill,outline=outline,width=width)
box(0,0,165.1,139.7,'#e2e8ec','#456173',3,11)
for rx in [9.85,88.55]:
    box(rx-3.15,77,73,51,'#f0d9d7','#b85e54',2,6)
    box(rx-1.0,119,68.7,6,'#c9dce6','#6190a7',1,2)
    p=xy(rx+33.35,101)
    text(*p,'RELAY',22,fill='#8d453e',bold=True,anchor='mm')
    p=xy(rx+33.35,93)
    text(*p,'66.7 x 45 mm',17,fill='#8d453e',anchor='mm')
box(48.26,14,68.58,53.34,'#d3e9e7','#408a84',2,5)
# Approximate connector direction indicators, not part of print.
box(46.2,45,7,8,'#b9c6cb','#65858b',1)
box(45.2,17,12,10,'#a7b7bd','#65858b',1)
p=xy(82.55,42); text(*p,'ARDUINO UNO',21,fill='#286a64',bold=True,anchor='mm')
p=xy(82.55,34); text(*p,'USB / power face left',15,fill='#286a64',anchor='mm')
for item in data['hole_coordinates']:
    cx,cy=xy(item['x'],item['y'])
    r=3*scale
    d.ellipse((cx-r,cy-r,cx+r,cy+r),fill='#86a7b9',outline='#35596d',width=2)
    r=scale
    d.ellipse((cx-r,cy-r,cx+r,cy+r),fill='white',outline='#35596d',width=1)

# Overall dimension lines.
x1,y1=xy(0,139.7); x2,y2=xy(165.1,139.7)
dy=275
d.line((x1,dy,x2,dy),fill='#526777',width=2)
for x in [x1,x2]:
    d.line((x,dy-8,x,dy+8),fill='#526777',width=2)
    d.line((x,dy+15,x,y1-7),fill='#8999a5',width=1)
text((x1+x2)/2,dy-26,'165.1 mm / 6.5 in',21,anchor='mm')
d.line((1715,y1,1715,oy),fill='#526777',width=2)
for y in [y1,oy]: d.line((1707,y,1723,y),fill='#526777',width=2)
# Height label horizontally beneath drawing avoids unreadable rotated type.
text(1060,872,'Height: 139.7 mm / 5.5 in',22,bold=True)
text(1060,911,'12 standoffs: 6 mm diameter x 2 mm tall',21)
text(1060,945,'2 mm through holes  |  3 mm base',21)
text(1060,992,'Colored board outlines are placement guides.',17,fill='#627782')
text(1060,1017,'UNO pattern assumed for the pictured Freenove V5.',17,fill='#627782')
# Clear caption on render field, below the object.
d.rounded_rectangle((72,932,920,1028),radius=12,fill='#e6edf1')
text(96,946,'Actual STL geometry',22,bold=True)
text(96,981,'Print flat, standoffs up. Total printed height: 5 mm.',21)
im.save(ROOT/'Mount_preview.png')
