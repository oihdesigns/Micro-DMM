"""Join the design BOM to the user's purchase history without modifying it."""
import csv,json
from pathlib import Path
import openpyxl
ROOT=Path(__file__).resolve().parent
source=Path(r'C:\Users\Nick\Dropbox (Personal)\AvaliableParts\Combined_parts_DigiKey_and_Mouser.xlsx')
w=openpyxl.load_workbook(source,read_only=True,data_only=True)
sheet=w['Combined parts']
inventory={str(row[0]).strip():(i,row) for i,row in enumerate(sheet.values,1) if row[0]}
parts=json.loads((ROOT/'circuit.json').read_text())
values={'100R':'RNCF0805DTE100R','75k / 0.1%':'ERA-6AEB753V','10k':'RN73H2ATTD1002B25','10k / 0.1%':'RN73H2ATTD1002B25','1M / 0.1%':'RG2012P-105-B-T5','15k / 0.1%':'RG2012P-153-B-T5','100n / 25V':'CL21B104KACNNNC','1u / 50V':'CL21B105KBFNNNE','1n / 50V':'C0805C102K5HACAUTO','10p / 1kV':'C0805C100JDGACTU','CONTROL / ADC':'Generic 1x10 2.54mm vertical header','SOURCE':'691311400102','LOAD':'691311400102'}
rows=[]
for p in parts:
    mpn=values.get(p['value'],p['value']);alt='AMC3330QDWERQ1' if mpn=='AMC3330DWER' else ''
    lookup=mpn if mpn in inventory else alt
    hit=inventory.get(lookup)
    rows.append(dict(reference=p['reference'],value=p['value'],manufacturer_part_number=mpn,stocked_alternative=alt,inventory_match='Exact MPN' if mpn in inventory else ('Compatible automotive variant; see README' if hit else 'Not found in purchase history'),inventory_sheet_row=hit[0] if hit else '',historical_quantity=hit[1][7] if hit else '',footprint=p['footprint'],purpose=p['purpose'],datasheet=p['datasheet']))
for name in ['BOM.csv','BOM_inventory.csv']:
    with (ROOT/name).open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
report={'source':str(source),'sheet':sheet.title,'quantity_note':'Historical ordered/received quantities; not a physical stock count. Shared parts consume stock cumulatively.','exact_reference_matches':sum(r['inventory_match']=='Exact MPN' for r in rows),'alternative_reference_matches':sum(bool(r['stocked_alternative']) for r in rows),'not_found_references':[r['reference'] for r in rows if r['inventory_match'].startswith('Not found')],'total_components':len(rows)}
(ROOT/'validation'/'inventory_summary.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
