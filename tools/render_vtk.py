import sys, numpy as np, vtk, math
from vtk.util import numpy_support as ns
import carl_cad as c
import cadquery as cq

def poly(shape, tol=0.06, ang=0.12):
    v,t=shape.val().tessellate(tol,ang)
    V=np.array([(p.x,p.y,p.z) for p in v],float); T=np.array(t,np.int64)
    pd=vtk.vtkPolyData(); pts=vtk.vtkPoints(); pts.SetData(ns.numpy_to_vtk(V,deep=True)); pd.SetPoints(pts)
    cells=np.hstack([np.full((len(T),1),3),T]).ravel()
    ca=vtk.vtkCellArray(); ca.SetCells(len(T),ns.numpy_to_vtkIdTypeArray(cells,deep=True)); pd.SetPolys(ca)
    return pd

def split_orange(pd, cond):
    """cells whose centroid satisfies cond(x,y,z) -> separate polydata"""
    V=ns.vtk_to_numpy(pd.GetPoints().GetData()); T=ns.vtk_to_numpy(pd.GetPolys().GetData()).reshape(-1,4)[:,1:]
    cen=V[T].mean(1); m=cond(cen[:,0],cen[:,1],cen[:,2])
    def mk(sel):
        sub=T[sel]; p=vtk.vtkPolyData(); pts=vtk.vtkPoints(); pts.SetData(ns.numpy_to_vtk(V,deep=True)); p.SetPoints(pts)
        ca=vtk.vtkCellArray(); ca.SetCells(len(sub),ns.numpy_to_vtkIdTypeArray(np.hstack([np.full((len(sub),1),3),sub]).ravel(),deep=True)); p.SetPolys(ca); return p
    return mk(~m), mk(m)

ren=vtk.vtkRenderer()
BLACK=(0.075,0.077,0.086); ORANGE=(1.0,0.38,0.04)
def add(pd, color, metal=0.0, rough=0.5, smooth=True):
    if smooth:
        n=vtk.vtkPolyDataNormals(); n.SetInputData(pd); n.SetFeatureAngle(35); n.SplittingOn(); n.ConsistencyOn(); n.AutoOrientNormalsOff(); n.Update(); pd=n.GetOutput()
    m=vtk.vtkPolyDataMapper(); m.SetInputData(pd); a=vtk.vtkActor(); a.SetMapper(m)
    p=a.GetProperty(); p.SetInterpolationToPBR(); p.SetColor(*color); p.SetMetallic(metal); p.SetRoughness(rough)
    ren.AddActor(a); return a

# ---- printed parts: black PETG; orange only on the lettering, bumper and pivot bushes
for n,(s,tf,note) in c.parts.items():
    pd=poly(s)
    if n.startswith("12_"):                      # side skirts: lettering stands proud of x=+-83
        base,txt=split_orange(pd, lambda x,y,z: np.abs(x)>83.05)
        add(base,BLACK,0.0,0.42); add(txt,ORANGE,0.0,0.35)
    elif n.startswith("14_"):
        base,strip=split_orange(pd, lambda x,y,z: (z>46.3)&(y>149.5))
        add(base,BLACK,0.0,0.45); add(strip,ORANGE,0.0,0.4)
    elif n.startswith("18_"):
        add(pd,ORANGE,0.0,0.4)
    else:
        add(pd,BLACK,0.0,0.45)

# ---- other hardware
def has(n,*k): return any(x.lower() in n.lower() for x in k)
for n,s,st in c.simple:
    if st=='d' or has(n,'Styling: cover post') and False: continue
    if has(n,'Cold-shoe','Board pad','Styling'):  add(poly(s),BLACK,0,0.45); continue
    if has(n,'wheel'):
        bb=s.val().BoundingBox(); xc=(bb.xmin+bb.xmax)/2; yc=(bb.ymin+bb.ymax)/2; zc=(bb.zmin+bb.zmax)/2
        tyre=c.cyl("X",50,40,(xc,yc,zc)).cut(c.cyl("X",31,42,(xc,yc,zc)))
        gr=None
        for i in range(36):
            g=c.box(xc-19,xc+19,yc-1.6,yc+1.6,zc+47.0,zc+51).rotate((xc,yc,zc),(xc+1,yc,zc),i*10)
            tyre=tyre.cut(g)
        add(poly(tyre,0.05,0.08),(0.025,0.025,0.028),0,0.85)
        rim=c.cyl("X",33,38,(xc,yc,zc)); add(poly(rim,0.05,0.08),(0.16,0.165,0.18),0.8,0.35)
        sgn=1 if xc>0 else -1
        cap=c.cyl("X",11,3,(xc+sgn*20.5,yc,zc)); add(poly(cap,0.05,0.08),(0.25,0.26,0.28),0.9,0.3)
        continue
    if has(n,'Motor can','Motor gearbox'): add(poly(s,0.05,0.08),(0.55,0.56,0.58),0.9,0.35); continue
    if has(n,'Motor shaft','Motor bush','standoff'): add(poly(s,0.05,0.08),(0.7,0.72,0.75),1.0,0.28); continue
    if has(n,'damper'): add(poly(s),(0.02,0.02,0.02),0,0.9); continue
    if has(n,'Sony a6700'): add(poly(s),(0.03,0.03,0.035),0.1,0.5); continue
    if has(n,'lens'): add(poly(s,0.05,0.06),(0.02,0.02,0.022),0.1,0.35); continue
    if has(n,'UNO Q'): add(poly(s),(0.03,0.2,0.24),0,0.5); continue
    if has(n,'MG996R','servo horn'): add(poly(s),(0.07,0.075,0.09),0.2,0.5); continue
    if has(n,'Ball head','Arca','bearing'): add(poly(s,0.05,0.08),(0.2,0.2,0.22),0.9,0.35); continue
    if has(n,'BTS7960','VL53','UBEC','buck','MPU','LiPo','switch','fuse','LED','USB'): add(poly(s),(0.06,0.07,0.09),0.1,0.55); continue
    add(poly(s),(0.1,0.1,0.11),0.2,0.5)

# ground: unlit textured plane (soft pool of light and a contact shadow), edges equal the background
N=1024; R=1200.0
xs=np.linspace(-R,R,N); X,Y=np.meshgrid(xs,xs)
r=np.hypot(X,Y); pool=np.clip(1-r/R,0,1)**1.6
sh=np.exp(-(np.maximum(np.abs(X)-128,0)**2+np.maximum(np.abs(Y)-185,0)**2)/(2*40.0**2))
lum=0.045+0.12*pool; lum=lum*(1-0.85*sh)
img=vtk.vtkImageData(); img.SetDimensions(N,N,1)
arr=np.repeat((np.clip(lum,0,1)*255).astype(np.uint8)[:,:,None],3,2).reshape(-1,3)
va=ns.numpy_to_vtk(arr,deep=True,array_type=vtk.VTK_UNSIGNED_CHAR); va.SetNumberOfComponents(3); img.GetPointData().SetScalars(va)
tex=vtk.vtkTexture(); tex.SetInputData(img); tex.InterpolateOn()
fl=vtk.vtkPlaneSource(); fl.SetOrigin(-R,-R,-0.3); fl.SetPoint1(R,-R,-0.3); fl.SetPoint2(-R,R,-0.3); fl.Update()
m=vtk.vtkPolyDataMapper(); m.SetInputConnection(fl.GetOutputPort()); g=vtk.vtkActor(); g.SetMapper(m); g.SetTexture(tex)
gp=g.GetProperty(); gp.SetInterpolationToFlat(); gp.SetAmbient(1.0); gp.SetDiffuse(0.0); gp.SetSpecular(0.0); ren.AddActor(g)

ren.GradientBackgroundOn(); ren.SetBackground(0.045,0.047,0.053); ren.SetBackground2(0.045,0.047,0.053)
def light(pos,intensity,color=(1,1,1),fp=(0,0,120)):
    l=vtk.vtkLight(); l.SetLightTypeToSceneLight(); l.SetPosition(*pos); l.SetFocalPoint(*fp); l.SetIntensity(intensity); l.SetColor(*color); l.SetPositional(False); ren.AddLight(l)
light((-700,800,900),2.0,(1,0.97,0.93)); light((900,500,500),0.9,(0.8,0.9,1.0)); light((300,-900,400),1.5,(1,0.85,0.7)); light((0,0,1500),0.6)
ren.UseImageBasedLightingOff()

w=vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(ren); w.SetMultiSamples(0)
W,H=int(sys.argv[2]) if len(sys.argv)>2 else 2000, int(sys.argv[3]) if len(sys.argv)>3 else 1250
w.SetSize(W,H)
# passes: SSAO + FXAA
basic=vtk.vtkRenderStepsPass()
ssao=vtk.vtkSSAOPass(); ssao.SetDelegatePass(basic); ssao.SetRadius(25); ssao.SetBias(0.5); ssao.SetKernelSize(128); ssao.BlurOn()
ren.SetPass(ssao)
ren.UseFXAAOn()

def shot(path, az, el, dist, focal=(0,0,118), zoom=1.0):
    cam=ren.GetActiveCamera(); a,e=math.radians(az),math.radians(el)
    cam.SetFocalPoint(*focal); cam.SetPosition(focal[0]+dist*math.cos(e)*math.sin(a), focal[1]+dist*math.cos(e)*math.cos(a), focal[2]+dist*math.sin(e))
    cam.SetViewUp(0,0,1); cam.SetViewAngle(24); ren.ResetCameraClippingRange(); cam.Zoom(zoom)
    w.Render()
    f=vtk.vtkWindowToImageFilter(); f.SetInput(w); f.ReadFrontBufferOff(); f.Update()
    p=vtk.vtkPNGWriter(); p.SetFileName(path); p.SetInputConnection(f.GetOutputPort()); p.Write()

out="/mnt/user-data/outputs/carlG/"
which=sys.argv[1] if len(sys.argv)>1 else "hero"
if which=="hero": shot(out+"carl_render_front.png",-38,16,980)
elif which=="all":
    shot(out+"carl_render_front.png",-38,16,980); shot(out+"carl_render_rear.png",148,17,1500); shot(out+"carl_render_side.png",-90,6,1700,zoom=1.0)
