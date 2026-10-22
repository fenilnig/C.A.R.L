import carl_cad as c, itertools
names=list(c.parts); sol={n:c.parts[n][0].val() for n in names}
def inter(a,b):
    try: return a.intersect(b).Volume()
    except Exception as e: return -1
print("PRINTED vs PRINTED overlaps (mm3):")
for a,b in itertools.combinations(names,2):
    ba,bb=sol[a].BoundingBox(),sol[b].BoundingBox()
    if ba.xmax<bb.xmin or bb.xmax<ba.xmin or ba.ymax<bb.ymin or bb.ymax<ba.ymin or ba.zmax<bb.zmin or bb.zmax<ba.zmin: continue
    v=inter(sol[a],sol[b])
    if v>1: print(f"  {a} x {b}: {v:.0f}")
print("PRINTED vs OTHER parts:")
for n,s,st in c.simple:
    bs=s.val().BoundingBox()
    for a in names:
        ba=sol[a].BoundingBox()
        if ba.xmax<bs.xmin or bs.xmax<ba.xmin or ba.ymax<bs.ymin or bs.ymax<ba.ymin or ba.zmax<bs.zmin or bs.zmax<ba.zmin: continue
        v=inter(sol[a],s.val())
        if v>1: print(f"  {a} x {n[:44]} [{st}]: {v:.0f}")
