"""Read-only parts-history lookup; original workbook is never modified."""
import json,re
from pathlib import Path
import openpyxl
source=Path(r'C:\Users\Nick\Dropbox (Personal)\AvaliableParts\Combined_parts_DigiKey_and_Mouser.xlsx')
w=openpyxl.load_workbook(source,read_only=True,data_only=True)
s=w['Combined parts']
rows=list(s.values)
for i,row in enumerate(rows[:8],1):print(i,row)
matches=[]
for i,row in enumerate(rows,1):
    if re.search(r'AMC|ACS|8751|SCT01|MOSFET|0805|LM358|TLV|OPA|LM324', ' '.join(str(x or '') for x in row),re.I):
        matches.append({'row':i,'values':row});print(i,row)
Path('validation/inventory_matches.json').write_text(json.dumps({'source':str(source),'sheet':s.title,'rows':matches},indent=2,default=str))
