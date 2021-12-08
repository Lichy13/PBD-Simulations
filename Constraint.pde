
abstract class Constraint{

  ArrayList<Particle> particles;
  float stiffness;    // k en el paper de Muller
  float k_coef;       // k' en el paper de Muller
  float C;
  
  Constraint(){
    particles = new ArrayList<Particle>();
  }
  
  void  compute_k_coef(int n){
    k_coef = 1.0 - pow((1.0-stiffness),1.0/float(n));
    //println("Fijamos "+n+" iteraciones   -->  k = "+stiffness+"    k' = "+k_coef+".");
  }

  abstract void proyecta_restriccion();
  abstract void display(float scale_px);
}

class DistanceConstraint extends Constraint{

  float d;
  
  DistanceConstraint(Particle p1,Particle p2,float dist,float k){
    super();
    d = dist;
    particles.add(p1);
    particles.add(p2);
    stiffness = k;
    k_coef = stiffness;
    C=0;

  }
  
  void proyecta_restriccion(){
    Particle part1 = particles.get(0); 
    Particle part2 = particles.get(1);
    
    PVector vd = PVector.sub(part1.location,part2.location);
    float dist = vd.mag();
    float C = dist - d;
    PVector n = PVector.div(vd,dist);
    PVector delta = PVector.mult(n, C);
    float wsp1 = -(part1.w/(part1.w + part2.w));
    float wsp2 = (part2.w/(part1.w + part2.w));
    if(debug){
      println("PROYECTA: p1="+part1.location);
      println("PROYECTA: p2="+part2.location);
      println("PROYECTA: p1-p2="+vd);
    }
    
    part1.location.add(PVector.mult(PVector.mult(delta,wsp1),k_coef));
    part2.location.add(PVector.mult(PVector.mult(delta,wsp2),k_coef));
    
    /**
     * COMPLETAR RESTRICCION
     * */
    
    
  }
  
  void display(float scale_px){
    PVector p1 = particles.get(0).location; 
    PVector p2 = particles.get(1).location; 
    strokeWeight(3);
    stroke(255,255*(1-abs(4*C/d)),255*(1-abs(4*C/d)));
    line(scale_px*p1.x, -scale_px*p1.y, scale_px*p1.z,  scale_px*p2.x, -scale_px*p2.y, scale_px*p2.z);
  };
  
}

class FoldConstraint extends Constraint{

  PVector centroide;
  float h0;
  
  FoldConstraint(Particle v,Particle b0,Particle b1,float k){
    super();
    particles.add(v);
    particles.add(b0);
    particles.add(b1);
    stiffness = k;
    centroide = PVector.div(PVector.add(v.location, PVector.add(b0.location, b1.location)),3);
    h0 = PVector.sub(v.location, centroide).mag();
    k_coef = stiffness;


  }
  
  void proyecta_restriccion(){
    Particle v = particles.get(0); 
    Particle b0 = particles.get(1);
    Particle b1 = particles.get(2);
    centroide = PVector.div(PVector.add(v.location, PVector.add(b0.location, b1.location)),3);
    float W = 2*v.w + b0.w + b1.w;
    float distoCenter = PVector.sub(v.location,centroide).mag();
    if(distoCenter != 0)
    {
       PVector vc = PVector.sub(v.location, centroide);
       PVector Varb0 = PVector.mult(vc,(2*b0.w/W)*(1-(h0/distoCenter)));
       PVector Varb1 = PVector.mult(vc,(2*b1.w/W)*(1-(h0/distoCenter)));
       PVector Varv = PVector.mult(vc,-(4*v.w/W)*(1-(h0/distoCenter)));
       b0.location.add(PVector.mult(Varb0, k_coef));
       b1.location.add(PVector.mult(Varb1, k_coef));
       v.location.add(PVector.mult(Varv, k_coef));
    }
    
    
    /**
     * COMPLETAR RESTRICCION
     * */
    
    
  }
  
  void display(float scale_px){
    PVector v = particles.get(0).location; 
    PVector b0 = particles.get(1).location;
    PVector b1 = particles.get(2).location;
    strokeWeight(3);
    stroke(255,255,255);
    line(scale_px*v.x, -scale_px*v.y, scale_px*v.z,  scale_px*b0.x, -scale_px*b0.y, scale_px*b0.z);
    line(scale_px*b0.x, -scale_px*b0.y, scale_px*b0.z,  scale_px*b1.x, -scale_px*b1.y, scale_px*b1.z);
    line(scale_px*b1.x, -scale_px*b1.y, scale_px*b1.z,  scale_px*v.x, -scale_px*v.y, scale_px*v.z);
  };
  
}

abstract class StaticCollisionConstraint
{
  
  PVector normal;
  ArrayList<Particle> particles;
  float stiffness;    // k en el paper de Muller
  float k_coef;       // k' en el paper de Muller
  
  StaticCollisionConstraint()
  {
    particles = new ArrayList<Particle>();
  }
  void  compute_k_coef(int n){
    k_coef = 1.0 - pow((1.0-stiffness),1.0/float(n));
    //println("Fijamos "+n+" iteraciones   -->  k = "+stiffness+"    k' = "+k_coef+".");
  }
  
  void update_vel_Collisions()
  {
    Particle par = particles.get(0);
    float nv = PVector.dot(normal,par.velocity);
    PVector vn = PVector.mult(normal,nv);
    PVector vt = PVector.sub(par.velocity, vn);
    par.velocity = PVector.sub(vt,vn);
  }
  
  abstract void proyecta_restriccion();
  abstract void display(float scale_px);
}

class FloorCollision extends StaticCollisionConstraint{

  float dcol;
  float rad;
  PVector pf;
  
  FloorCollision(Particle part,PVector floorpoint, PVector floornormal,float radius, float k){
    super();
    particles.add(part);
    stiffness = k;
    k_coef = stiffness;
    normal = floornormal;
    rad = radius;
    pf = floorpoint;
    dcol = PVector.sub(part.location, floorpoint).dot(normal);
  }
  
  void proyecta_restriccion(){
    Particle part = particles.get(0); 
    dcol = PVector.sub(part.location, pf).dot(normal);
    PVector variation = PVector.mult(normal, (dcol-rad));
    //println("Variation " + dcol);
    part.location.sub(PVector.mult(variation,k_coef));
    
  }
  
  void display(float scale_px){
    PVector part = particles.get(0).location; 
    strokeWeight(3);
    stroke(255,0,0);
    line(scale_px*part.x, -scale_px*part.y, scale_px*part.z,  scale_px*part.x, -scale_px*part.y - dcol, scale_px*part.z);
  };
  
}

class StaticSphereCollisionConstraint extends StaticCollisionConstraint{

  float distance;
  float radSphere;
  PVector pf;
  
  StaticSphereCollisionConstraint(Particle part,PVector Sphere, PVector Spherenormal,float radius, float k){
    super();
    particles.add(part);
    stiffness = k;
    k_coef = stiffness;
    normal = Spherenormal;
    radSphere = radius;
    pf = Sphere;
    distance = PVector.sub(pf, part.location).mag();
  }
  
  void proyecta_restriccion(){
    Particle part = particles.get(0); 
    distance = PVector.sub(part.location, pf).mag();
    PVector variation = PVector.mult(normal, (distance-radSphere));
    println("Variation " + distance);
    part.location.sub(PVector.mult(variation,k_coef));
    
    
    
    /**
     * COMPLETAR RESTRICCION
     * */
    
    
  }
  
  void update_vel_Collisions()
  {
    Particle par = particles.get(0);
    float nv = PVector.dot(normal,par.velocity);
    PVector vn = PVector.mult(normal,nv);
    PVector vt = PVector.sub(par.velocity, vn);
    par.velocity = PVector.sub(vt,vn);
  }
  
  void display(float scale_px){
    PVector part = particles.get(0).location; 
    strokeWeight(3);
    stroke(255,0,0);
    line(scale_px*part.x, -scale_px*part.y, scale_px*part.z,  scale_px*part.x, -scale_px*part.y - distance, scale_px*part.z);
  };
  
}
