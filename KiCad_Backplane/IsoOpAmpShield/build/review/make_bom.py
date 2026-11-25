"""Build mfg/<TS>/IsoOpAmpShield/bom.csv + positions.csv (fab LCSC codes; customer-fitted parts left out)."""
import sys,csv,re
from collections import OrderedDict
TS=sys.argv[1]; D=f"mfg/{TS}/IsoOpAmpShield"
sys.argv=["x"]; exec(open("build/review/pads.py").read())
info={}
for f in kids(root,"footprint"):
    pr={p[1]:p[2] for p in kids(f,"property") if len(p)>2}
    attr=one(f,"attr"); attr=attr[1:] if attr else []
    info[pr["Reference"]]=dict(val=pr.get("Value",""),fp=f[1].split(":")[-1],pn=pr.get("PN",""),attr=attr)
M={
"12065C102KAT2A":("12065C102KAT2A","KYOCERA AVX","C597498"),
"AMC3301DWER":("AMC3301DWER","Texas Instruments","C2869778"),
"AMC3330DWER":("AMC3330DWER","Texas Instruments","C2999368"),
"OPA2192IDR":("OPA2192IDR","Texas Instruments","C110074"),
"C0603C102K5RACTU":("C0603C102K5RACTU","KEMET","C2167129"),
"C0603C103K5RACTU":("C0603C103K5RACTU","KEMET","C116378"),
"C0603C104K5RACAUTO":("C0603C104K5RACAUTO","KEMET","C129621"),
"C0603C331J5GACTU":("C0603C331J5GACTU","KEMET","C452750"),
"C0805C102J5GACTU":("C0805C102J5GACTU","KEMET","C2167084"),
"CL21A226MAYNNNE":("CL21A226MAYNNNE","Samsung Electro-Mechanics","C602037"),
"CL21A475KAQNNNG":("CL21A475KAQNNNG","Samsung Electro-Mechanics","C722528"),
"ERJ-6ENF1000V":("ERJ-6ENF1000V","Panasonic","C193990"),
"ERJ-6ENF1002V":("ERJ-6ENF1002V","Panasonic","C169253"),
"ERJ-6ENF8252V":("ERJ-6ENF8252V","Panasonic","C3025615"),
"LQH32PN100MN0L":("LQH32PN100MN0L","Murata","C98020"),
"TPS7A3001DGNR":("TPS7A3001DGNR","Texas Instruments","C33835"),
"TPS7A4901DGNR":("TPS7A4901DGNR","Texas Instruments","C16430"),
# fab-stock substitutes for part numbers it doesn't carry
"12063C105KAT2A":("CL31B105KBHNNNE","Samsung Electro-Mechanics","C1848"),
"74269244182":("BLM15HD182SN1D","Murata","C76888"),
"C0603C103J5GACTU":("GRM1885C1H103JA01D","Murata","C85973"),
"C0805X103JCRAC3316":("0805CG103J500NT","FH (Fenghua)","C2971995"),
"CGA3E1X7R1E105K080AC":("CL10B105KA8NNNC","Samsung Electro-Mechanics","C29936"),
"CGA5L2NP01H104J160AA":("GRM31C5C1H104JA01L","Murata","C97946"),
"CRT0805ABY-10R0ELF":("RT0805BRD0710RL","YAGEO","C865142"),
"ERA-6AED4993V":("RT0805BRD07499KL","YAGEO","C706243"),
"ERJ-8ENF1003V":("1206W4F1003T5E","UNI-ROYAL","C17900"),
"HVC1206Z1008KET":("1206W4F1006T5E","UNI-ROYAL","C59781"),
"RG2012P-103-B-T5":("RT0805BRD0710KL","YAGEO","C110775"),
"RG2012P-104-B-T5":("RT0805BRD07100KL","YAGEO","C122537"),
"RG2012P-153-B-T5":("RT0805BRD0715KL","YAGEO","C728642"),
"RG2012P-4991-B-T5":("RT0805BRD074K99L","YAGEO","C706246"),
"TBJ05-02-1-G-G":("DB128L-5.08-2P-GN-S","DORABO","C395868"),
}
SKIP={"PS1","J1","J2"}  # fitted by the customer
rows=OrderedDict(); fitted=set(); skipped=[]
key=lambda r:(re.match(r"[A-Z]+",r).group(),int(re.search(r"\d+",r).group()))
for ref in sorted(info,key=key):
    i=info[ref]
    if "dnp" in i["attr"] or "exclude_from_bom" in i["attr"] or ref in SKIP: skipped.append(ref); continue
    mpn,mfr,lcsc=M[i["pn"]]
    r=rows.setdefault(mpn,dict(refs=[],val=i["val"],fp=i["fp"],mpn=mpn,mfr=mfr,lcsc=lcsc)); r["refs"].append(ref); fitted.add(ref)
with open(f"{D}/bom.csv","w",newline="",encoding="utf8") as fh:
    w=csv.writer(fh); w.writerow(["Designator","Quantity","Value","Footprint","MPN","Manufacturer","LCSC"])
    for r in rows.values(): w.writerow([",".join(r["refs"]),len(r["refs"]),r["val"],r["fp"],r["mpn"],r["mfr"],r["lcsc"]])
n=0
with open("build/review/pos_raw.csv",encoding="utf8") as fi, open(f"{D}/positions.csv","w",newline="",encoding="utf8") as fo:
    w=csv.writer(fo); w.writerow(["Designator","Mid X","Mid Y","Rotation","Layer"])
    for r in csv.DictReader(fi):
        if r["Ref"] in fitted:
            # Mid X/Y = centre of the part's pads (the fab's models are centred); KiCad's anchor is pin 1 on
            # THT connectors such as J3-J5. pos export flips Y, so the centre goes out as -y.
            ps=[p for p in pads if p["ref"]==r["Ref"]]
            cx=sum(p["x"] for p in ps)/len(ps); cy=sum(p["y"] for p in ps)/len(ps)
            w.writerow([r["Ref"],f"{cx:.4f}",f"{-cy:.4f}",r["Rot"],"Top" if r["Side"]=="top" else "Bottom"]); n+=1
print("bom lines",len(rows),"fitted",len(fitted),"positions",n,"not assembled:",skipped)
