import fitz,sys
# crop.py out.png x0 y0 x1 y1 (KiCad mm) [dpi]
x0,y0,x1,y1=map(float,sys.argv[2:6]); dpi=int(sys.argv[6]) if len(sys.argv)>6 else 500
d=fitz.open('build/pcb_render.svg'); p=d[0]
sx=p.rect.width/101.6; sy=p.rect.height/53.34
r=fitz.Rect((x0-100)*sx,(y0-100)*sy,(x1-100)*sx,(y1-100)*sy)
p.get_pixmap(dpi=dpi,clip=r).save(sys.argv[1])
