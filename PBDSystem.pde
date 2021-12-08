class PBDSystem {
  // Partículas del sistema
  ArrayList<Particle> particles;
  ArrayList<Constraint> constraints;
  ArrayList<StaticCollisionConstraint> CollisionConstraints;
  Floor floor;
  boolean ActiveFloor = false;
  PVector SpherePosition = new PVector(ancho_tela/2,-alto_tela/2,-profundo_tela/2);
  float SphereRadius = 0.2;
  boolean ActiveSphere = false;
  
  int niters;


  PBDSystem(int n,float mass) {
    //array de particulas luminosas.Aun NO SE CREAN las particulas concretas
    particles = new ArrayList<Particle>(n);
    constraints = new ArrayList<Constraint>();
    CollisionConstraints = new ArrayList<StaticCollisionConstraint>();
    floor = new Floor();
    niters = 5;
    
    PVector p = new PVector(0,0,0);
    PVector v = new PVector(0,0,0);
 
    for(int i=0;i<n;i++){
      particles.add(new Particle(p,v,mass));
    }
  }

  void set_n_iters(int n){
   niters = n;
   for(int i = 0; i< constraints.size(); i++)
     constraints.get(i).compute_k_coef(n);
  }

  void add_constraint(Constraint c){
     constraints.add(c);
     c.compute_k_coef(niters);
  }
  
  void add_collision_constraint(StaticCollisionConstraint c){
     CollisionConstraints.add(c);
     c.compute_k_coef(niters);
  }
  
  void add_collision_sphere_constraint(StaticSphereCollisionConstraint c){
     CollisionConstraints.add(c);
     c.compute_k_coef(niters);
  }

  void apply_gravity(PVector g){
    Particle p;
    for(int i = 0 ; i<particles.size() ; i++){
      p = particles.get(i);
      if(p.w > 0) // Si la masa en infinito, no se aplica
        p.force.add(PVector.mult(g,p.masa));
    }
  }

  void run(float dt) {
    //Simulacion
    for (int i = 0; i < particles.size(); i++)
      particles.get(i).update(dt);
      
    for(int i = 0; i < particles.size(); i++)
    {
      if(ActiveFloor)
      {
        if(floor.IsColliding(particles.get(i)))
        {
          StaticCollisionConstraint c = new FloorCollision(particles.get(i),floor.x1,floor.normal,particles.get(i).display_size, 1);
          this.add_collision_constraint(c);
          //println(i);
        }  
      }
      
      if(ActiveSphere)
      {
        if(PVector.sub(SpherePosition,particles.get(i).location).mag() < particles.get(i).display_size+SphereRadius)
        {
          StaticCollisionConstraint c = new StaticSphereCollisionConstraint(particles.get(i),SpherePosition,PVector.sub(SpherePosition,particles.get(i).location).normalize(),particles.get(i).display_size, 0.05);
          this.add_collision_constraint(c);
          println("Collision Con Esfera");
        }  
      }
      
    }
      

    for(int it = 0; it< niters; it++)
    {
      for(int i = 0; i < CollisionConstraints.size(); i++)
        CollisionConstraints.get(i).proyecta_restriccion();
      for(int i = 0; i< constraints.size(); i++)
        constraints.get(i).proyecta_restriccion();
      
    }
     
    for (int i = 0; i < particles.size(); i++)
      particles.get(i).update_pbd_vel(dt);
      
    for (int i = 0; i < CollisionConstraints.size(); i++)
      CollisionConstraints.get(i).update_vel_Collisions();
    
    CollisionConstraints.clear();
  }

}
