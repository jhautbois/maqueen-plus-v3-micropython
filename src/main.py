"""Obstacle Avoidance with LIDAR, Memory System & Corner Escape"""
from microbit import display, Image, button_a, button_b, sleep, running_time
import music
from maqueen_plus_v3 import MaqueenPlusV3
from laser_matrix import LaserMatrix

# Thresholds (mm)
CRIT=150; STOP=250; SLOW=400; SAFE=600
# Speed
SPD_MIN=100; SPD_MAX=180; TSPD=100; RSPD=70
# Memory
HIST_SZ=8; STUCK_MS=4000; ESC_TURN=500; ESC_REV=400

SP=[(0,0),(2,0),(5,0),(7,0),(0,2),(2,2),(5,2),(7,2),
    (3,3),(4,3),(3,4),(4,4),(0,5),(2,5),(5,5),(7,5),
    (0,7),(2,7),(5,7),(7,7)]

def d2b(d):
    if d<200: return 9
    if d<400: return 7
    if d<600: return 5
    if d<1000: return 3
    if d<1500: return 1
    return 0

class App:
    def __init__(self):
        display.show(Image.HEART)
        self.r = MaqueenPlusV3()
        self.l = LaserMatrix(0x33)
        self.l.set_mode(8)
        # LED test
        print("Testing LEDs...")
        self.r.headlights('red','red'); sleep(400)
        self.r.headlights('green','green'); sleep(400)
        self.r.headlights('white','white')
        for _ in range(3):
            self.r.underglow('all',255,0,0); sleep(100)
        self.r.underglow('all',0,255,0)
        # State
        self.on=False; self.mv=False; self.la="s"
        # Memory system
        self.hist=[0]*HIST_SZ; self.hidx=0
        self.last_prog=0; self.stuck_cnt=0
        self.escaping=False; self.esc_dir=1; self.esc_start=0
        display.show(Image.HAPPY)
        print("Ready A=go")

    def rec(self,d,t):
        """Record direction: -1=L,0=straight,1=R,2=reverse"""
        self.hist[self.hidx]=d
        self.hidx=(self.hidx+1)%HIST_SZ
        if d==0: self.last_prog=t; self.stuck_cnt=0

    def is_osc(self):
        """Detect L/R oscillation"""
        alt=0; p=None
        for d in self.hist:
            if d in(-1,1):
                if p is not None and d!=p: alt+=1
                p=d
        return alt>=4

    def is_stuck(self,t):
        """Detect if stuck"""
        no_prog=(t-self.last_prog)>STUCK_MS
        rev_cnt=sum(1 for d in self.hist if d==2)
        return no_prog or rev_cnt>=3 or self.is_osc()

    def calc_esc(self,z):
        """Calculate escape direction (opposite of tendency)"""
        lc=sum(1 for d in self.hist if d==-1)
        rc=sum(1 for d in self.hist if d==1)
        if lc>rc+2: return 1
        if rc>lc+2: return -1
        if z['fl']>z['fr']+200: return -1
        if z['fr']>z['fl']+200: return 1
        self.esc_dir=-self.esc_dir
        return self.esc_dir

    def rsp(self):
        p={}
        for x,y in SP:
            d=self.l.read_point(x,y)
            p[(x,y)]=d if d>0 else 4000
        return p

    def rfm(self):
        m=self.l.read_matrix()
        p={}
        if m:
            for y in range(8):
                for x in range(8):
                    p[(x,y)]=m[y*8+x]
        return p

    def zones(self,p):
        z={'fl':[],'l':[],'c':[],'r':[],'fr':[]}
        for (x,y),d in p.items():
            if d>=4000: continue
            if x<=1: z['fl'].append(d)
            elif x<=2: z['l'].append(d)
            elif x<=4: z['c'].append(d)
            elif x<=5: z['r'].append(d)
            else: z['fr'].append(d)
        r={}
        for k,v in z.items():
            r[k]=sum(v)//len(v) if v else 4000
        return r

    def disp(self,p):
        rows=[]
        for dy in range(5):
            row=""
            for dx in range(5):
                lx,ly=int((4-dx)*1.6),int((4-dy)*1.6)
                ds=[]
                for i in range(2):
                    for j in range(2):
                        k=(min(lx+i,7),min(ly+j,7))
                        if k in p: ds.append(p[k])
                row+=str(d2b(sum(ds)//len(ds) if ds else 4000))
            rows.append(row)
        display.show(Image(":".join(rows)))

    def dir(self,z):
        fl,l,c,r,fr=z['fl'],z['l'],z['c'],z['r'],z['fr']
        ls,rs=fl*0.3+l*0.7, fr*0.3+r*0.7
        if c>SAFE:
            d=(rs-ls)/3000.0
            d=max(-0.3,min(0.3,d))
            cf=min(c/1000.0,1.0)
        elif c>STOP:
            d=-0.5 if ls>rs else 0.5
            cf=max(ls,rs)/1000.0
        else:
            d=-1.0 if ls>rs else 1.0
            cf=max(ls,rs)/1500.0
        return (d,max(0.0,min(1.0,cf)))

    def leds(self,cf,a):
        if a in("rv","esc"): c=(255,0,0)
        elif a=="t": c=(0,0,255)
        elif cf<0.3: c=(255,100,0)
        elif cf<0.6: c=(0,150,150)
        else: c=(0,255,0)
        self.r.underglow('all',*c)

    def move(self,d,cf,z):
        t=running_time()
        c=z['c']

        # Escape mode active?
        if self.escaping:
            dur=ESC_TURN*min(self.stuck_cnt,3)
            if t-self.esc_start<dur:
                self.r.turn(self.esc_dir*TSPD)
                self.leds(0,"esc")
                return True
            self.escaping=False
            self.hist=[0]*HIST_SZ; self.hidx=0

        # Check if stuck
        if self.is_stuck(t):
            self.stuck_cnt+=1
            self.escaping=True
            self.esc_start=t
            self.esc_dir=self.calc_esc(z)
            print("STUCK! esc="+str(self.esc_dir))
            music.pitch(600,200)
            self.r.drive(-RSPD); self.leds(0,"esc")
            sleep(ESC_REV*min(self.stuck_cnt,3))
            self.r.turn(self.esc_dir*TSPD)
            self.la="esc"
            return True

        # Record direction
        if d<-0.3: dd=-1
        elif d>0.3: dd=1
        else: dd=0
        self.rec(dd,t)

        # Critical: reverse
        if c<CRIT:
            self.rec(2,t)
            music.pitch(800,100)
            self.r.drive(-RSPD); self.leds(0,"rv"); sleep(400)
            td=1 if d>=0 else -1
            self.r.turn(td*TSPD); sleep(350)
            self.la="rv"; return True

        # Close: turn
        if c<STOP:
            td=1 if d>=0 else -1
            self.r.turn(td*TSPD)
            self.leds(cf,"t"); self.la="t"
            return True

        # Clear path: reset stuck counter
        if c>SAFE and cf>0.7:
            self.last_prog=t; self.stuck_cnt=0

        # Adaptive speed
        if c>1200: spd=SPD_MAX
        elif c>SAFE: spd=SPD_MIN+int((SPD_MAX-SPD_MIN)*(c-SAFE)/(1200-SAFE))
        else: spd=SPD_MIN

        if abs(d)>0.3:
            if d<0: self.r.motors(int(spd*(1+d)),spd)
            else: self.r.motors(spd,int(spd*(1-d)))
            self.la="st"
        else:
            self.r.drive(spd); self.la="dr"
        self.leds(cf,self.la)
        return True

    def btn(self):
        if button_a.was_pressed():
            self.on=not self.on
            if self.on:
                self.last_prog=running_time()
                display.show(Image.YES)
            else:
                self.r.stop(); display.show(Image.NO)
            sleep(300)
        if button_b.was_pressed():
            self.on=False; self.r.stop()
            display.show(Image.SKULL); sleep(500)

    def run(self):
        while True:
            self.btn()
            if not self.on:
                p=self.rfm()
                if p: self.disp(p)
                sleep(100); continue
            p=self.rsp() if self.mv and self.la not in["rv","t","esc"] else self.rfm()
            if not p: self.r.stop(); continue
            self.disp(p)
            z=self.zones(p)
            d,cf=self.dir(z)
            self.mv=self.move(d,cf,z)

try:
    App().run()
except Exception as e:
    print("ERR:"+str(e))
    display.show(Image.SAD)
