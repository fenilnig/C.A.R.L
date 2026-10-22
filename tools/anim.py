import sys, math, numpy as np, vtk
from vtk.util import numpy_support as ns
import carl_cad as c
from PIL import Image, ImageDraw, ImageFont

FPS=24; DUR=14.0; NF=int(FPS*DUR); W,H=1280,720
L=260.0; KP=76.0
# ------------------------------------------------------------------ simulation (all in mm, deg)
def smooth(t,a,b):
    x=min(max((t-a)/(b-a),0),1); return x*x*(3-2*x)
def v_of(t):  return 250*smooth(t,3.0,4.5)*(1-smooth(t,10.8,12.2))
def dc_of(t): return math.radians(13*smooth(t,5.5,7.0)-13*smooth(t,8.0,9.0)-10*smooth(t,9.0,10.2)+10*smooth(t,10.2,11.2))   # centre steering angle
dt=1.0/FPS
S=[]; pos=np.array([0.0,0.0]); th=0.0; ph={k:0.0 for k in ("fl","fr","rl","rr")}; pan=0.0; tilt=0.0
person0=None; pan_f=0.0; tilt_f=0.0
for i in range(NF):
    t=i*dt; v=v_of(t); dc=dc_of(t)
    w=v*math.tan(dc)/L
    th+=w*dt; pos=pos+v*dt*np.array([-math.sin(th),math.cos(th)])
    # wheel angles
    if abs(math.tan(dc))>1e-6:
        R=L/math.tan(dc); dl=math.atan(L/(R-104.0)); dr=math.atan(L/(R+104.0))
        # signed: for left turn (R>0) left wheel is inner (larger angle)
    else: dl=dr=0.0
    vl=v+w*(-104.0); vr=v+w*(104.0)
    ph["rl"]-=vl/50*dt; ph["rr"]-=vr/50*dt; ph["fl"]-=vl/50*dt; ph["fr"]-=vr/50*dt
    gx=pos+np.array([-math.sin(th),math.cos(th)])*15.0      # gimbal axis in world (car-local y=15)
    # person
    pers=None
    if t>=5.0:
        if person0 is None:
            fwd=np.array([-math.sin(th),math.cos(th)]); right=np.array([math.cos(th),math.sin(th)])
            person0=pos+fwd*900-right*200
        pers=person0+np.array([70.0,215.0])*(t-5.0)
    # camera head
    if t<3.0:
        pan_t=45*math.sin(2*math.pi*t/1.5) if t<1.5 else 0.0
        tilt_t=22*math.sin(2*math.pi*(t-1.5)/1.5) if t>=1.5 else 0.0
        pan_f,tilt_f=pan_t,tilt_t
    else:
        if pers is None: pan_t,tilt_t=0.0,0.0
        else:
            d=pers-gx; ang=math.degrees(math.atan2(-d[0],d[1]))-math.degrees(th)
            pan_t=max(-80,min(80,(ang+180)%360-180)); tilt_t=math.degrees(math.atan2(160,np.hypot(*d)))
        k=1-math.exp(-dt/0.3); pan_f+=(pan_t-pan_f)*k; tilt_f+=(tilt_t-tilt_f)*k
    S.append(dict(t=t,v=v,dc=dc,pos=pos.copy(),th=th,dl=dl,dr=dr,ph=dict(ph),pan=pan_f,tilt=tilt_f,pers=None if pers is None else pers.copy()))
final=S[-1]; fwd=np.array([-math.sin(final['th']),math.cos(final['th'])])
OBST=final['pos']+fwd*(154+230+60)

# ------------------------------------------------------------------ scene
ren=vtk.vtkRenderer()
BLACK=(0.075,0.077,0.086); ORANGE=(1.0,0.38,0.04)
ACT=[]
def poly(shape,tol=0.06,ang=0.12):
    vv,tt=shape.val().tessellate(tol,ang)
    V=np.array([(p.x,p.y,p.z) for p in vv],float); T=np.array(tt,np.int64)
    pd=vtk.vtkPolyData(); pts=vtk.vtkPoints(); pts.SetData(ns.numpy_to_vtk(V,deep=True)); pd.SetPoints(pts)
    ca=vtk.vtkCellArray(); ca.SetCells(len(T),ns.numpy_to_vtkIdTypeArray(np.hstack([np.full((len(T),1),3),T]).ravel(),deep=True)); pd.SetPolys(ca); return pd
def split(pd,cond):
    V=ns.vtk_to_numpy(pd.GetPoints().GetData()); T=ns.vtk_to_numpy(pd.GetPolys().GetData()).reshape(-1,4)[:,1:]
    cen=V[T].mean(1); m=cond(cen[:,0],cen[:,1],cen[:,2])
    def mk(sel):
        sub=T[sel]; p=vtk.vtkPolyData(); pts=vtk.vtkPoints(); pts.SetData(ns.numpy_to_vtk(V,deep=True)); p.SetPoints(pts)
        ca=vtk.vtkCellArray(); ca.SetCells(len(sub),ns.numpy_to_vtkIdTypeArray(np.hstack([np.full((len(sub),1),3),sub]).ravel(),deep=True)); p.SetPolys(ca); return p
    return mk(~m),mk(m)
def add(pd,color,tag,metal=0.0,rough=0.5,smooth_=True):
    if smooth_:
        n=vtk.vtkPolyDataNormals(); n.SetInputData(pd); n.SetFeatureAngle(35); n.SplittingOn(); n.ConsistencyOn(); n.AutoOrientNormalsOff(); n.Update(); pd=n.GetOutput()
    m=vtk.vtkPolyDataMapper(); m.SetInputData(pd); a=vtk.vtkActor(); a.SetMapper(m)
    p=a.GetProperty(); p.SetInterpolationToPBR(); p.SetColor(*color); p.SetMetallic(metal); p.SetRoughness(rough)
    ren.AddActor(a); ACT.append((a,tag)); return a
def has(n,*k): return any(x.lower() in n.lower() for x in k)

for n,(s,tf,note) in c.parts.items():
    pd=poly(s); tag="body"
    if n.startswith(("15_","16_","18_")): tag="pan"
    elif n.startswith("17_"): tag="tilt"
    elif n.startswith("09_") or n.startswith("10_"): tag="steer"+("L" if n.endswith("L") else "R")
    elif n.startswith("11_"): tag="tie"
    if n.startswith("12_"):
        base,txt=split(pd,lambda x,y,z: np.abs(x)>83.05); add(base,BLACK,tag,0,0.42); add(txt,ORANGE,tag,0,0.35)
    elif n.startswith("14_"):
        base,strip=split(pd,lambda x,y,z:(z>46.3)&(y>149.5)); add(base,BLACK,tag,0,0.45); add(strip,ORANGE,tag,0,0.4)
    elif n.startswith("18_"): add(pd,ORANGE,tag,0,0.4)
    else: add(pd,BLACK,tag,0,0.45)
for n,s,st in c.simple:
    if st=='d': continue
    tag="body"
    if has(n,'MG996R tilt servo'): tag="pan"
    elif has(n,'Ball head','Arca','Sony a6700','Sony 16-50','USB vision camera (hot-shoe)','Cold-shoe'): tag="tilt"
    if has(n,'wheel'):
        bb=s.val().BoundingBox(); xc=(bb.xmin+bb.xmax)/2; yc=(bb.ymin+bb.ymax)/2; zc=(bb.zmin+bb.zmax)/2
        front=yc>0; side="L" if xc<0 else "R"; tg=("fw" if front else "rw")+side
        tyre=c.cyl("X",50,40,(xc,yc,zc)).cut(c.cyl("X",31,42,(xc,yc,zc)))
        for i in range(36): tyre=tyre.cut(c.box(xc-19,xc+19,yc-1.6,yc+1.6,zc+47.0,zc+51).rotate((xc,yc,zc),(xc+1,yc,zc),i*10))
        add(poly(tyre,0.05,0.08),(0.03,0.03,0.034),tg,0,0.85)
        add(poly(c.cyl("X",33,38,(xc,yc,zc)),0.05,0.08),(0.16,0.165,0.18),tg,0.8,0.35)
        sg=1 if xc>0 else -1
        add(poly(c.cyl("X",11,3,(xc+sg*20.5,yc,zc)),0.05,0.08),(0.25,0.26,0.28),tg,0.9,0.3); continue
    if has(n,'Cold-shoe','Board pad','Styling'): add(poly(s),BLACK,tag,0,0.45)
    elif has(n,'Motor can','Motor gearbox'): add(poly(s,0.05,0.08),(0.55,0.56,0.58),tag,0.9,0.35)
    elif has(n,'Motor shaft','Motor bush','standoff'): add(poly(s,0.05,0.08),(0.7,0.72,0.75),tag,1.0,0.28)
    elif has(n,'damper'): add(poly(s),(0.02,0.02,0.02),tag,0,0.9)
    elif has(n,'Sony a6700'): add(poly(s),(0.03,0.03,0.035),tag,0.1,0.5)
    elif has(n,'lens'): add(poly(s,0.05,0.06),(0.02,0.02,0.022),tag,0.1,0.35)
    elif has(n,'UNO Q'): add(poly(s),(0.03,0.2,0.24),tag,0,0.5)
    elif has(n,'MG996R','servo horn'): add(poly(s),(0.07,0.075,0.09),tag,0.2,0.5)
    elif has(n,'Ball head','Arca','bearing'): add(poly(s,0.05,0.08),(0.2,0.2,0.22),tag,0.9,0.35)
    else: add(poly(s),(0.06,0.07,0.09),tag,0.1,0.55)

# soft contact shadow under the car
for r,op in ((1.0,0.05),(0.86,0.06),(0.72,0.07),(0.58,0.08)):
    d=vtk.vtkCubeSource(); d.SetXLength(300*r+40); d.SetYLength(430*r+40); d.SetZLength(0.2); d.SetCenter(0,0,0.15)
    m=vtk.vtkPolyDataMapper(); m.SetInputConnection(d.GetOutputPort()); a=vtk.vtkActor(); a.SetMapper(m)
    p=a.GetProperty(); p.SetColor(0,0,0); p.SetOpacity(op); p.SetAmbient(1); p.SetDiffuse(0); p.SetSpecular(0); ren.AddActor(a); ACT.append((a,"shadow"))

# the subject: a simple stand-in figure
def part(src,color,tag,rough=0.6):
    m=vtk.vtkPolyDataMapper(); m.SetInputConnection(src.GetOutputPort()); a=vtk.vtkActor(); a.SetMapper(m)
    p=a.GetProperty(); p.SetInterpolationToPBR(); p.SetColor(*color); p.SetRoughness(rough); p.SetMetallic(0.0); ren.AddActor(a); ACT.append((a,tag)); a.SetVisibility(False); return a
def cylsrc(r,h,z0):
    s=vtk.vtkCylinderSource(); s.SetRadius(r); s.SetHeight(h); s.SetResolution(40); s.SetCenter(0,h/2+z0,0); s.Update()
    tr=vtk.vtkTransform(); tr.RotateX(90); f=vtk.vtkTransformPolyDataFilter(); f.SetInputConnection(s.GetOutputPort()); f.SetTransform(tr); return f
legs=part(cylsrc(22,190,0),(0.30,0.31,0.34),"person"); torso=part(cylsrc(34,150,190),(0.78,0.79,0.82),"person")
hs=vtk.vtkSphereSource(); hs.SetRadius(26); hs.SetCenter(0,0,372); hs.SetThetaResolution(40); hs.SetPhiResolution(40); head=part(hs,(0.78,0.79,0.82),"person")
ob=vtk.vtkCubeSource(); ob.SetXLength(150); ob.SetYLength(60); ob.SetZLength(150); ob.SetCenter(0,0,75); obst=part(ob,(0.5,0.5,0.52),"obst")

# floor: textured grid with a soft pool of light
N=2048; R=3500.0
xs=np.linspace(-R,R,N); X,Y=np.meshgrid(xs,xs)
pool=np.clip(1-np.hypot(X,Y-900)/R,0,1)**1.7
grid=((np.abs((X+50)%200-100)<2.2)|(np.abs((Y+50)%200-100)<2.2)).astype(float)
lum=0.045+0.11*pool+0.05*pool*grid
img=vtk.vtkImageData(); img.SetDimensions(N,N,1)
arr=np.repeat((np.clip(lum,0,1)*255).astype(np.uint8)[:,:,None],3,2).reshape(-1,3)
va=ns.numpy_to_vtk(arr,deep=True,array_type=vtk.VTK_UNSIGNED_CHAR); va.SetNumberOfComponents(3); img.GetPointData().SetScalars(va)
tex=vtk.vtkTexture(); tex.SetInputData(img); tex.InterpolateOn(); tex.MipmapOn()
fl=vtk.vtkPlaneSource(); fl.SetOrigin(-R,-R,-0.3); fl.SetPoint1(R,-R,-0.3); fl.SetPoint2(-R,R,-0.3); fl.Update()
m=vtk.vtkPolyDataMapper(); m.SetInputConnection(fl.GetOutputPort()); g=vtk.vtkActor(); g.SetMapper(m); g.SetTexture(tex)
gp=g.GetProperty(); gp.SetInterpolationToFlat(); gp.SetAmbient(1.0); gp.SetDiffuse(0.0); gp.SetSpecular(0.0); ren.AddActor(g)
ren.SetBackground(0.045,0.047,0.053)
def light(pos,i,col=(1,1,1)):
    l=vtk.vtkLight(); l.SetLightTypeToSceneLight(); l.SetPosition(*pos); l.SetFocalPoint(0,0,0); l.SetIntensity(i); l.SetColor(*col); ren.AddLight(l)
light((-700,800,900),2.0,(1,0.97,0.93)); light((900,500,500),0.9,(0.8,0.9,1.0)); light((300,-900,400),1.5,(1,0.85,0.7)); light((0,0,1500),0.6)
w=vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(ren); w.SetMultiSamples(0); w.SetSize(W,H)
ssao=vtk.vtkSSAOPass(); ssao.SetDelegatePass(vtk.vtkRenderStepsPass()); ssao.SetRadius(25); ssao.SetBias(0.5); ssao.SetKernelSize(96); ssao.BlurOn(); ren.SetPass(ssao); ren.UseFXAAOn()

# ------------------------------------------------------------------ transforms
def Tm(x,y,z): m=np.eye(4); m[:3,3]=(x,y,z); return m
def Rz(a): c_,s_=math.cos(a),math.sin(a); m=np.eye(4); m[0,0]=c_; m[0,1]=-s_; m[1,0]=s_; m[1,1]=c_; return m
def Rx(a): c_,s_=math.cos(a),math.sin(a); m=np.eye(4); m[1,1]=c_; m[1,2]=-s_; m[2,1]=s_; m[2,2]=c_; return m
def about(M,p): return Tm(*p)@M@Tm(*(-np.array(p)))
def setm(a,M):
    mm=vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4): mm.SetElement(i,j,M[i,j])
    a.SetUserMatrix(mm)
TILT_Z=c.TILT_Z; GY=c.GY
CAP={1.0:None}
cam_focal=None
def captions(t):
    for a,b,txt in ((0.0,3.0,"1  PAN + TILT  ·  the camera head moves on its own"),(3.0,5.5,"2  DRIVE  ·  gentle ramp up, 1 m/s cap"),(5.5,8.0,"3  STEER  ·  Ackermann front, differential rear"),(8.0,11.5,"4  TRACK  ·  pan and tilt follow the subject"),(11.5,14.1,"5  SAFETY  ·  ToF sensor brakes before the obstacle")):
        if a<=t<b: return txt
    return ""
def font(sz,bold=False):
    return ImageFont.truetype("/usr/share/fonts/opentype/inter/Inter-%s.otf"%("SemiBold" if bold else "Regular"),sz)
FT=font(30,True); FS=font(20); FM=font(18)

def frame(i):
    f=S[i]; t=f['t']; th=f['th']; pos=f['pos']
    World=Tm(pos[0],pos[1],0)@Rz(th)
    Pan=about(Rz(math.radians(f['pan'])),(0,GY,0)); Tilt=about(Rx(math.radians(f['tilt'])),(0,GY,TILT_Z))
    for a,tag in ACT:
        if tag=="body": setm(a,World)
        elif tag=="pan": setm(a,World@Pan)
        elif tag=="tilt": setm(a,World@Pan@Tilt)
        elif tag=="shadow": setm(a,World)
        elif tag=="tie": setm(a,World@Tm(14.5*math.sin(f['dc']),0,0))
        elif tag in ("steerL","steerR"):
            d=f['dl'] if tag=="steerL" else f['dr']; kx=-KP if tag=="steerL" else KP
            setm(a,World@about(Rz(d),(kx,118,0)))
        elif tag in ("fwL","fwR"):
            d=f['dl'] if tag=="fwL" else f['dr']; kx=-KP if tag=="fwL" else KP; xc=-104 if tag=="fwL" else 104
            setm(a,World@about(Rz(d),(kx,118,0))@about(Rx(f['ph']["fl" if tag=="fwL" else "fr"]),(xc,130,50)))
        elif tag in ("rwL","rwR"):
            xc=-104 if tag=="rwL" else 104
            setm(a,World@about(Rx(f['ph']["rl" if tag=="rwL" else "rr"]),(xc,-130,50)))
        elif tag=="person":
            a.SetVisibility(f['pers'] is not None)
            if f['pers'] is not None:
                bob=3*math.sin((t-5)*9); setm(a,Tm(f['pers'][0],f['pers'][1],bob)@Rz(math.atan2(-70,215)))
        elif tag=="obst":
            a.SetVisibility(t>=8.5); setm(a,Tm(OBST[0],OBST[1],0)@Rz(final['th']))
    global cam_focal
    tgt=np.array([pos[0],pos[1],105.0])
    if f['pers'] is not None: tgt=np.array([pos[0]+0.18*(f['pers'][0]-pos[0]),pos[1]+0.18*(f['pers'][1]-pos[1]),105.0])
    cam_focal=tgt if cam_focal is None else cam_focal+(tgt-cam_focal)*0.2
    az=math.radians(-52+26*smooth(t,0,14)); dist=1300+500*smooth(t,3,9); el=math.radians(20)
    cam=ren.GetActiveCamera()
    cam.SetFocalPoint(*cam_focal)
    cam.SetPosition(cam_focal[0]+dist*math.cos(el)*math.sin(az), cam_focal[1]-dist*math.cos(el)*math.cos(az), cam_focal[2]+dist*math.sin(el))
    cam.SetViewUp(0,0,1); cam.SetViewAngle(26); ren.ResetCameraClippingRange()
    w.Render()
    fl_=vtk.vtkWindowToImageFilter(); fl_.SetInput(w); fl_.ReadFrontBufferOff(); fl_.Update()
    im=fl_.GetOutput(); a=ns.vtk_to_numpy(im.GetPointData().GetScalars()).reshape(H,W,-1)[::-1,:,:3]
    pil=Image.fromarray(a.copy()); dr=ImageDraw.Draw(pil,"RGBA")
    cap=captions(t)
    if cap:
        dr.rounded_rectangle((28,24,28+dr.textlength(cap,font=FT)+36,24+54),12,fill=(8,9,11,170)); dr.text((46,34),cap,font=FT,fill=(236,236,238))
    hud="SPEED %.2f m/s    STEER %+.0f°    PAN %+.0f°    TILT %+.0f°"%(f['v']/1000,math.degrees(f['dc']),f['pan'],f['tilt'])
    dr.rounded_rectangle((28,H-66,28+dr.textlength(hud,font=FS)+36,H-26),10,fill=(8,9,11,170)); dr.text((46,H-58),hud,font=FS,fill=(214,216,220))
    tag="C.A.R.L.  concept animation, simulated, not real footage"
    dr.text((W-28-dr.textlength(tag,font=FM),H-52),tag,font=FM,fill=(150,152,158))
    return pil

if __name__=="__main__":
    a,b=int(sys.argv[1]),int(sys.argv[2]); out=sys.argv[3]
    for i in range(a,min(b,NF)): frame(i).save("%s/f%04d.png"%(out,i))
