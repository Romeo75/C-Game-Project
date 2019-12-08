#include <cstdlib>
#include <iostream>
#include <sstream>
#include <math.h>
#include <vector>
#include <chrono>
#include <SFML/Graphics.hpp> // un seul include suffit pour avoir les trois parties essentielles
						     // de la SFML: graphics, window et system

using namespace std;
using namespace sf;

//Function for time reference it marks the begining of the counter
auto started = chrono::high_resolution_clock::now();

class space_object {

    private:

    	string name;

        virtual void dynamics(int n, double t, double y[], double dy[]){

    		dy[0] = y[1];
    		dy[1] = y[2];

		}

    public:

        double MaxSpeed;
		double sizeX,sizeY;
        static int windowSizeX;
        static int windowSizeY;
        int life;
        double gravity;
    
        double x[3],y[3]; /*Canonical vectors, range 0 is the position of the object,
                    ranges above are the derivates in time
                    */
                   
        /// TEXTURES \\\

        Sprite shape;
        Texture texture;

        //Defined by partner for rotation mouvement rho and phy 
        double trust,phy; //provisonal

        string GetName(){ return name;}
        void SetName(string name){ this->name = name;}
        
        double GetSizeX(){ return sizeX;}
        double GetSizeY(){ return sizeY;}

        bool isDead(){ return (life <= 0); }
        
        double GetMaxSpeed(){ return MaxSpeed;}

        void SetSize(int sizeX,int sizeY){
            this->sizeX = sizeX;
            this->sizeX = sizeX;
        }

        void draw (	RenderWindow &window){ window.draw(shape);}

        void SetLimits(int windowSizeX, int windowSizeY,double MaxSpeed){

            this->windowSizeX = windowSizeX;
            this->windowSizeY = windowSizeY;
            this->MaxSpeed = MaxSpeed;

        }
        
        int GetWindowSizeX(){ return windowSizeX;}
        int GetWindowSizeY(){ return windowSizeY;}

        //Clock to get the time since execution (it returns elapsed time in ms)
        double GetTickCount(){
            auto done = chrono::high_resolution_clock::now();
            return chrono::duration_cast<chrono::milliseconds>(done-started).count();
        }

        virtual void ApplyLimits(){
            /*
            This sets the limits of the space object
            */

            //Deffines Spacial limits
            if ( x[0] >= (windowSizeX-sizeX*0.5) ){
                x[1]=-x[1];
                x[0]=windowSizeX-sizeX*0.5;
            }

            if ( x[0] < sizeX*0.5 ) {
                x[1]=-x[1];
                x[0]=sizeX*0.5;
            } 
        
            if ( y[0] >= (windowSizeY-sizeY*0.5) ){
                y[1]=-y[1];
                y[0]=windowSizeY-sizeY*0.5;
            }

            if ( y[0] < sizeY*0.5 ) {
                y[1]=-y[1];
                y[0]=sizeY*0.5;
            }

            //Deffines Speed limits
            if ( x[1] > abs(MaxSpeed) ){
                x[1]=MaxSpeed;
            }

            if ( y[1] > abs(MaxSpeed) ){
                y[1]=MaxSpeed;
            }
            
            if ( x[1] < -abs(MaxSpeed) ){
                x[1]=-MaxSpeed;
            }

            if ( y[1] < -abs(MaxSpeed) ){
                y[1]=-MaxSpeed;
            }

        }

        double * GetVectorX(){ return x;}
        double * GetVectorY(){ return y;}

        double GetDirectionX() { return cos( phy*0.017453f );}
        double GetDirectionY() { return sin( phy*0.017453f );}

        void SetPosition(double xSpeed,double ySpeed){
            this->x[0] = xSpeed;
            this->y[0] = ySpeed;
        }

        void SetSpeed(double xSpeed,double ySpeed){
            this->x[1] = xSpeed;
            this->y[1] = ySpeed;
        }
        void SetForces(double xForce,double yForce){
            this->x[2] = xForce;
            this->y[2] = yForce;
        }

        double distance(space_object& A){

            double dx2 = pow((A.GetVectorX()[0]-GetVectorX()[0]), 2);
            double dy2 = pow((A.GetVectorY()[0]-GetVectorY()[0]), 2);

            return sqrt(dx2 + dy2);

        }

        void rk4(int n, double x, double y[], double dx){

            /*-----------------------------------------
            sous programme de resolution d'equations
            differentielles du premier ordre par
            la methode de Runge-Kutta d'ordre 4
            x = abscisse
            y = valeurs des fonctions
            dx = pas
            n = nombre d'equations differentielles
            dynamics = variable contenant le nom du
            sous-programme qui calcule les derivees
            ----------------------------------------*/

            int i ;
            double ddx ;
            /* d1, d2, d3, d4 = estimations des derivees
            yp = estimations intermediaires des fonctions */
            double d1[n], d2[n], d3[n], d4[n], yp[n];
            
            ddx = dx/2;                /* demi-pas */
            
            dynamics(n,x,y,d1) ;          /* 1ere estimation */          
            
            for( i = 0; i< n; i++){ yp[i] = y[i] + d1[i]*ddx ; }
            dynamics(n,x+ddx,yp,d2) ;     /* 2eme estimat. (1/2 pas) */
            
            for( i = 0; i < n; i++){ yp[i] = y[i] + d2[i]*ddx ; }
            dynamics(n,x+ddx,yp,d3) ; /* 3eme estimat. (1/2 pas) */
            
            for( i = 0; i< n; i++){ yp[i] = y[i] + d3[i]*dx ;}
            dynamics(n,x+dx,yp,d4) ;      /* 4eme estimat. (1 pas) */
            /* estimation de y pour le pas suivant en utilisant
            une moyenne pondérée des dérivées en remarquant
            que : 1/6 + 1/3 + 1/3 + 1/6 = 1 */
            
            for( i = 0; i < n ; i++)
                { y[i] += dx*( d1[i] + 2*d2[i] + 2*d3[i] + d4[i] )/6 ; }
        }
        
        virtual void UpdatePosition(){
            
            rk4(3, 0., x, 1e-1);
            rk4(3, 0., y, 1e-1);
            ApplyLimits();
            shape.setPosition(x[0],y[0]);
            shape.setRotation(phy - 90);

        }


        void SetAll(string name, int life, double sizeX, double sizeY, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[]);
        
        virtual void GetAll();

        //Constructors
        space_object(string name, double gravity, int life, Color color, string picture, double sizeX, double sizeY, int windowSizeX, int windowSizeY, double MaxSpeed, double x[], double y[], double phy);
        space_object(int windowSizeX, int windowSizeY);

        //Destructor
        ~space_object(); // Frees up memory after objects are destroyed (To be defined - investigate)

};

//Instance of Static Variables

    int space_object::windowSizeX = 800;
    int space_object::windowSizeY = 600;


//Methods related to Space

    void space_object::SetAll(string name, int life, double sizeX, double sizeY, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[]){

        this->name = name;
        this->life = life;
        this->sizeX = sizeX;
        this->sizeY = sizeY;
        this->windowSizeX = windowSizeX;
        this->windowSizeY = windowSizeY;
        this->MaxSpeed = MaxSpeed;
        for(int i=0; i<3; i ++) this-> x[i] = x[i];
        for(int i=0; i<3; i ++) this-> y[i] = y[i];

    }

    void space_object::GetAll(){

        //Check control
        cout<< endl
            << "Name: " << name << endl
            << "Life: " << life << endl
            <<  "WindowSizeX: " << windowSizeX << endl
            <<  "WindowSizeY: " << windowSizeY << endl
            <<  "MaxSpeed:  "    << MaxSpeed << endl
            <<  "   Rotation:   " << phy << endl
            <<  "   Position: ("    << x[0]<< ","<<y[0]<<")"
            <<  "   Vitesse: ("     << x[1]<< ","<<y[1]<<")"
            <<  "   acceleration: ("<< x[2]<< ","<<y[2]<<")\n";
    }

    space_object::space_object(string name, double gravity, int life, Color color, string picture, double sizeX, double sizeY, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[], double phy){

        this-> name = name;
        this-> life = life;
        this-> gravity = gravity;
        this-> sizeX = sizeX;
        this-> sizeY = sizeY;
        this-> windowSizeX = windowSizeX;
        this-> windowSizeY = windowSizeY;
        this-> MaxSpeed = MaxSpeed;
        this-> phy = phy;
        for(int i=0; i<3; i ++) this-> x[i] = x[i];
        for(int i=0; i<3; i ++) this-> y[i] = y[i];
        this-> texture.loadFromFile(picture);
        this-> texture.setSmooth(true);
        this-> shape.setTextureRect(IntRect(0,0,sizeX,sizeY));
        this-> shape.setOrigin(sizeX/2.,sizeY/2.);
        this-> shape.setRotation(phy - 90);
        this-> shape.setTexture(texture);
    }

    space_object::space_object(int windowSizeX, int windowSizeY){

        this->name = "";
        this->sizeX = 10; // in px
        this->sizeY = 10;
        this->windowSizeX = windowSizeX; // in px
        this->windowSizeY = windowSizeY; // in px
        this->MaxSpeed = 60; // in px/s
        this-> x[0] = sizeX; this-> x[1] = 0.; this-> x[2] = 0.;
        this-> y[0] = sizeY; this-> y[1] = 0.; this-> y[2] = 0.;
    }
    space_object::~space_object(){ cout<< endl << name << " Destroyed...";}

//Class Shot that defines the object our spacecraft will create each time it fires
class Shot: public space_object {

	private:

        double ttl; // time to live of the shot

        double externalForces; // ? Force to take into account in the differential system of mouvement

        double createdTime;

	public:

        void UpdatePosition(){
            
            cout<< "\n........................BigLol........................"<<endl;
            rk4(3, 0., x, 1e-1);
            rk4(3, 0., y, 1e-1);
            ApplyLimits();
            shape.setPosition(x[0],y[0]);
            shape.setRotation(phy-90);

        }    

        void externalForce(space_object& a){
            
            cout << "distance de la planete: " << distance(a);

            x[2] += a.gravity * (a.x[0]-x[0])*pow(distance(a),-1);
            y[2] += a.gravity * (a.y[0]-y[0])*pow(distance(a),-1);

            
        }

        // Remove a bullet if its TTL has expired
        bool LivingTime(){
            return (GetTickCount() - createdTime >= ttl);
        }

        //Shot Consructor
        Shot(string name, double gravity, int life, Color color, string picture, double sizeX, double sizeY, double ttl, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[],double phy):space_object(name, gravity, life, color, picture, sizeX, sizeY, windowSizeX, windowSizeY, MaxSpeed , x, y,phy){
            this-> ttl = ttl;
            this-> createdTime = GetTickCount();
        }

        // Delete a bullet
        ~Shot(){}

};

//Class that defines players it inherits methodes and variables from space object class
class ship: public space_object{

	private:

 		void dynamics(int n, double t, double y[], double dy[]){

    		dy[0] = y[1];
    		dy[1] = y[2] - 0.7*y[1]; // -y[1] is a non conservative force (it makes the control of the spacecraft easier)

		}

        

    public:

        // Number of shots currently in use
        int shotsUsed;

        // Max number of allowed shots
        int maxShots;

        // Minimum time between each shot (ms)
        unsigned int shotCooldown;

        // Time last shot was fired
        unsigned int lastShotTime;

        bool firing;
	
        //Vectors of shots fired (Misils or Bullets?) And ships
        vector<Shot> ShotsInSpace;
        
        void GetAll(){

            //Check control
            cout << endl
                 <<  "Name: " << GetName() << endl
                 <<  "WindowSizeX: " << GetWindowSizeX() << endl
                 <<  "WindowSizeY: " << GetWindowSizeY() << endl
                 <<  "MaxSpeed:  "    << GetMaxSpeed() << endl
                 <<  "Shots fired vector size:  " << (ShotsInSpace.size()) << endl
                 <<  "Time (in ms):  "<< GetTickCount() << endl
                 <<  "   Rotation:   " << phy << endl
                 <<  "   Position: ("    << GetVectorX()[0]<< ","<<GetVectorY()[0]<<")"
                 <<  "   Vitesse: ("     << GetVectorX()[1]<< ","<<GetVectorY()[1]<<")"
                 <<  "   acceleration: ("<< GetVectorX()[2]<< ","<<GetVectorY()[2]<<")\n";
        }
        
        // Create a new shot and ensures that it doesn't goes like crazy
    	void Fire();

        // Release one shot slot
	    void EndFire();

	    // Allow a new shot to be fired immediately if any slots free
	    void ResetShotCooldown();

        //Add control over the ship
        virtual void GetInput(int sensibility);

        ship(string name, double gravity, int life, Color color, string picture, double sizeX,double sizeY, int maxShots, int windowSizeX, int windowSizeY, double MaxSpeed, double x[], double y[], double phy);
        ship(int windowSizeX, int windowSizeY):space_object(windowSizeX,windowSizeY){};
        ~ship();
};

//Methods related to ship objects

    void ship::Fire(){

            // Don't fire unless the cooldown period has expired
            if ((GetTickCount() - lastShotTime) >= shotCooldown)
            {
                
                // Don't fire if the maximum number of Shots are already on screen
                if (shotsUsed < maxShots)
                {

                    double x[]={GetVectorX()[0], GetDirectionX()*2e2, 0};
                    double y[]={GetVectorY()[0], GetDirectionY()*2e2, 0};
                    
                    // Makes new Shot
                    Shot shot( "Boom... ", 1., 3, Color::Black, "spaceMissil.png", 20, 35, 3e3, GetWindowSizeX(), GetWindowSizeY(), 2e2, x, y, phy + 180);
                    
                    //Stores it in a vector
                    (this->ShotsInSpace).push_back(shot);
                    
                    // Last Shot fired now!!!!
                    lastShotTime = GetTickCount();
                    shotsUsed++;

                }
            }
            
        }

    
    /*
        Stop firing a bullet (called back when a Bullet object is destroyed)
    */
    void ship::EndFire(){
        shotsUsed = max(shotsUsed - 1, 0);
    }

    // Reset cooldown (used when fire key is released)
    void ship::ResetShotCooldown(){
        lastShotTime = 0;
    }


    void ship::GetInput(int sensibility){

            if ( Keyboard::isKeyPressed(sf::Keyboard::Left) )  {        this->phy   += -sensibility;
                if ( Keyboard::isKeyPressed(sf::Keyboard::Up) ){        this->trust += sensibility;
                    if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                    else firing = false;
                }
                else if ( Keyboard::isKeyPressed(sf::Keyboard::Down) ){ this->trust += -sensibility;
                    if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                    else firing = false;
                }
                else if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                else firing = false;
            }

            else if ( Keyboard::isKeyPressed(sf::Keyboard::Right) ) {   this->phy   += +sensibility;
                if ( Keyboard::isKeyPressed(sf::Keyboard::Up) ){         this->trust += sensibility;
                    if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                    else firing = false;
                }
                else if ( Keyboard::isKeyPressed(sf::Keyboard::Down) ){  this->trust += -sensibility;
                    if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                    else firing = false;
                }
                else if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                else firing = false;
            }
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Up) ){        this->trust += sensibility;
                if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                else firing = false;
            }
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Down) ){      this->trust += -sensibility;
                if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                else firing = false;
            }

            else if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
            else {

                trust = 0.;
                this->GetVectorX()[2] = 0.;
                this->GetVectorY()[2] = 0.;
                ResetShotCooldown();
                firing = false;
            
            }

            GetVectorX()[2] = trust*GetDirectionX();
            GetVectorY()[2] = trust*GetDirectionY();

        }

    ship::ship(string name, double gravity, int life, Color color, string picture, double sizeX, double sizeY, int maxShots, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[],double phy):space_object(name, gravity, life, color, picture, sizeX, sizeY, windowSizeX, windowSizeY, MaxSpeed , x, y, phy){
        this->maxShots = maxShots;
        this->shotsUsed = 0;
        this->lastShotTime = GetTickCount();
        this->firing = false;
        this->shotCooldown = 500; // Time between each shot if fired continously (in ms)
    }
    ship::~ship(){}

//Defines the player 2
class ship2: public ship{
    private:

    public:

        void GetInput(int sensibility);

        ship2(string name, double gravity, int life, Color color, string picture, double sizeX,double sizeY, int maxShots, int windowSizeX, int windowSizeY, double MaxSpeed, double x[], double y[], double phy):ship( name, gravity, life, color, picture, sizeX, sizeY, maxShots, windowSizeX, windowSizeY, MaxSpeed, x, y, phy ){}
        ~ship2(){}
};

void ship2::GetInput(int sensibility){

    if ( Keyboard::isKeyPressed(sf::Keyboard::Q) )  {        this->phy   += -sensibility;
        if ( Keyboard::isKeyPressed(sf::Keyboard::Z) ){        this->trust += sensibility;
            if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
            else firing = false;
        }
        else if ( Keyboard::isKeyPressed(sf::Keyboard::S) ){ this->trust += -sensibility;
            if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
            else firing = false;
        }
        else if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
        else firing = false;
    }

    else if ( Keyboard::isKeyPressed(sf::Keyboard::D) ) {   this->phy   += +sensibility;
        if ( Keyboard::isKeyPressed(sf::Keyboard::Z) ){         this->trust += sensibility;
            if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
            else firing = false;
        }
        else if ( Keyboard::isKeyPressed(sf::Keyboard::S) ){  this->trust += -sensibility;
            if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
            else firing = false;
        }
        else if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
        else firing = false;
    }
    else if ( Keyboard::isKeyPressed(sf::Keyboard::Z) ){        this->trust += sensibility;
        if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
        else firing = false;
    }
    else if ( Keyboard::isKeyPressed(sf::Keyboard::S) ){      this->trust += -sensibility;
        if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
        else firing = false;
    }

    else if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
    else
    {
        trust = 0.;
        this->GetVectorX()[2] = 0.;
        this->GetVectorY()[2] = 0.;
        ResetShotCooldown();
        firing = false;
    }

    GetVectorX()[2] = trust*GetDirectionX();
    GetVectorY()[2] = trust*GetDirectionY();


}
//Class that defines the planets
class planet: public space_object{

    private:


    public:
        
        void UpdatePosition(){
                
                cout<<"\nLOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOL";
                rk4(3, 0., x, 1e-1);
                rk4(3, 0., y, 1e-1);
                ApplyLimits();
                shape.setPosition(x[0],y[0]);
                shape.setRotation(phy - 90);

            }

        double RandPosition(double sup){

            return rand()%(int)abs(sup)-0.5*sup;

        }

        planet(string name, double gravity, int life, Color color, string picture, double sizeX, double sizeY, int windowSizeX, int windowSizeY, double MaxSpeed, double x[], double y[], double phy):space_object( name, gravity, life, color, picture, sizeX, sizeY, windowSizeX, windowSizeY, MaxSpeed, x, y, phy){}
        ~planet(){}

};

int main(){


    /// Properties Of the Background\\\
        
        int windowSizeX = 800, windowSizeY = 600;
        RenderWindow window(VideoMode(windowSizeX, windowSizeY), "Spacecraft Movement");
        window.setFramerateLimit(40);
        Texture t1;
        t1.loadFromFile("blue.png");
        t1.setRepeated(true);
        Sprite sFond(t1,IntRect(0,0,windowSizeX,windowSizeY));

    /// Properties Of the Objects\\\
    
        double textX = 0,textY = 0;
        double sizeX = 100, sizeY = 94;     // dimensions of the ship
        double sizePX = 215, sizePY = 211;  // dimensions of the planet
        
        double gravity = 1e2;
        int life = 100;
        
        double x[]={windowSizeX/4.,0.,0.},y[]={windowSizeY*0.5,0.,0.}; // position initiale du veseau x[0],y[0], vitesse x[1], y[1] et acceleration x[3], x
                                                            // x[2] et y[2] Intensité des forces subies par le cercle exprimees dans la base canonique.
        int vmax = 100, maxShots = 10;
        double phy = 0;

    /// Objects \\\

        ship p("player1", gravity, life, Color::Green, "spaceShip.png", sizeX, sizeY, maxShots, windowSizeX, windowSizeY, vmax, x, y, phy);
        

        vector<planet> PlanetsInSpace;

        planet mars( "Mars", gravity, life, Color::Blue, "meteor.png", sizePX, sizePY, windowSizeX, windowSizeY, vmax, x, y, phy );
        
        x[0]=windowSizeX*3/4.;y[0]=windowSizeY*0.5;
        ship2 p2("player2", gravity, life, Color::Green, "spaceShip.png", sizeX, sizeY, maxShots, windowSizeX, windowSizeY, vmax, x, y, phy);

        planet moon("Moon", gravity, life, Color::Blue, "meteor.png", sizePX, sizePY, windowSizeX, windowSizeY, vmax, x, y, phy );
        PlanetsInSpace.push_back(mars);
        PlanetsInSpace.push_back(moon);
        
        Text PlayerLife;
        ostringstream dataPlayer1;
        
        RectangleShape boundShip0;
        RectangleShape boundShip1;

        

    while (window.isOpen()){

        Event event;

        while (window.pollEvent(event)){
            if (event.type == Event::Closed) window.close();
          }
        //Closses the window when you press (x)

        //Sets the background
		window.clear(Color::Black);
        window.draw(sFond);        

        //clears the terminal
        system("clear");    
    
        //Updates all information about planets

            for (int i = 0; i < (PlanetsInSpace).size(); i++)
            {
                PlanetsInSpace[i].GetAll();
                PlanetsInSpace[i].UpdatePosition();
                //PlanetsInSpace[i].SetForces(PlanetsInSpace[0].RandPosition( 10 ), PlanetsInSpace[0].RandPosition( 10 ));
            }
            cout<<"\nNumber of planets in Space: " << PlanetsInSpace.size();

        //Trackers
            //Player 1
            boundShip0.setOrigin(               p.shape.getOrigin().x,              p.shape.getOrigin().y);
            boundShip0.setPosition(             p.shape.getPosition().x,            p.shape.getPosition().y);
            boundShip0.setSize(sf::Vector2f(    p.shape.getGlobalBounds().width,    p.shape.getGlobalBounds().height));
            boundShip0.setFillColor(Color::Transparent);
            boundShip0.setOutlineThickness(5);
            boundShip0.setOutlineColor(Color::Magenta);
            window.draw(boundShip0);
            
            //Player 2
            boundShip1.setOrigin(               p2.shape.getOrigin().x,             p2.shape.getOrigin().y);
            boundShip1.setPosition(             p2.shape.getPosition().x,           p2.shape.getPosition().y);
            boundShip1.setSize(sf::Vector2f(    p2.shape.getGlobalBounds().width,   p2.shape.getGlobalBounds().height));
            boundShip1.setFillColor(Color::Transparent);
            boundShip1.setOutlineThickness(5);
            boundShip1.setOutlineColor(Color::Magenta);
            window.draw(boundShip1);



        //Player 1 Conditions    
            //Get the input from the arrows in the keyboard for player1
            p.GetInput(3);
            if ( p.firing ){
                
                cout<< endl << p.GetName() + "  Fireeee!!!!!!!";
                p.Fire();
                p.ShotsInSpace[0].GetAll();

            }
        
            //Sequence that updates all of the shots in space
            for (int i = 0 ; i < (p.ShotsInSpace).size(); i++){
                
                (p.ShotsInSpace[i]).UpdatePosition();    
                (p.ShotsInSpace[i]).texture.loadFromFile("spaceMissil.png");
                (p.ShotsInSpace[i]).shape.setTexture((p.ShotsInSpace[i]).texture);
                (p.ShotsInSpace[i]).GetAll();
                (p.ShotsInSpace[i]).phy = 180 + (180/M_PI) * acos(((p.ShotsInSpace[i]).x[1]) * pow( sqrt( pow(((p.ShotsInSpace[i]).x[1]), 2) + pow( ((p.ShotsInSpace[i]).y[1]), 2) ) ,-1));
                cout<< "\nPlayer 1:"
                    << "\n Angle par le Cosinus: "     << 180 + (180/M_PI) * acos(((p.ShotsInSpace[i]).x[1]) * pow( sqrt( pow(((p.ShotsInSpace[i]).x[1]), 2) + pow( ((p.ShotsInSpace[i]).y[1]), 2) ) ,-1)) 
                    << "\n Angle par le Sinus: "       << 180 + (180/M_PI) * asin(((p.ShotsInSpace[i]).y[1]) * pow( sqrt( pow(((p.ShotsInSpace[i]).x[1]), 2) + pow( ((p.ShotsInSpace[i]).y[1]), 2) ) ,-1));
                (p.ShotsInSpace[i]).SetForces(0.,0.);
                
                (p.ShotsInSpace[i]).draw(window);
                
                for (int l = 0 ; l < (PlanetsInSpace).size(); l++) if (!PlanetsInSpace[l].isDead()) (p.ShotsInSpace[i]).externalForce(PlanetsInSpace[l]);

                //If the shot is no longer permited then erase
                if( (*( (p.ShotsInSpace).begin()+i )).LivingTime()) {
                    p.EndFire();
                    (p.ShotsInSpace).erase((p.ShotsInSpace).begin()+ i);
                }
                else if ( !PlanetsInSpace[0].isDead() && ( (p.ShotsInSpace)[i].shape.getGlobalBounds().intersects( PlanetsInSpace[0].shape.getGlobalBounds() ) )){
                    PlanetsInSpace[0].life -= 10;
                    p.EndFire();
                    (p.ShotsInSpace).erase((p.ShotsInSpace).begin()+ i);
    
                }
                else if ( !PlanetsInSpace[1].isDead() && ( (p.ShotsInSpace)[i].shape.getGlobalBounds().intersects( PlanetsInSpace[1].shape.getGlobalBounds() ) )){
                    PlanetsInSpace[1].life -= 10;
                    p.EndFire();
                    (p.ShotsInSpace).erase((p.ShotsInSpace).begin()+ i);
                }
            }
            
            //Update position of all the elements related to the player p
            p.UpdatePosition();
            
            //Check control
            p.GetAll();

        //Player 2 Conditions
            //Get the input from the arrows in the keyboard for player1
            p2.GetInput(3);
            if ( p2.firing ){
                
                cout<< endl << p2.GetName() + " Fireeee!!!!!!!";
                p2.Fire();
                p2.ShotsInSpace[0].GetAll();

            }
        
            //Sequence that updates all of the shots in space
            for (int i = 0 ; i < (p2.ShotsInSpace).size(); i++){
                
                (p2.ShotsInSpace[i]).UpdatePosition();    
                (p2.ShotsInSpace[i]).texture.loadFromFile("spaceMissil.png");
                (p2.ShotsInSpace[i]).shape.setTexture((p2.ShotsInSpace[i]).texture);
                (p2.ShotsInSpace[i]).GetAll();
                (p2.ShotsInSpace[i]).phy = 180 + (180/M_PI) * acos(((p2.ShotsInSpace[i]).x[1]) * pow( sqrt( pow(((p2.ShotsInSpace[i]).x[1]), 2) + pow( ((p2.ShotsInSpace[i]).y[1]), 2) ) ,-1));
                cout<< "\nPlayer2:" 
                    << "\n Angle par le Cosinus: "     << 180 + (180/M_PI) * acos(((p2.ShotsInSpace[i]).x[1]) * pow( sqrt( pow(((p2.ShotsInSpace[i]).x[1]), 2) + pow( ((p2.ShotsInSpace[i]).y[1]), 2) ) ,-1)) 
                    << "\n Angle par le Sinus: "       << 180 + (180/M_PI) * asin(((p2.ShotsInSpace[i]).y[1]) * pow( sqrt( pow(((p2.ShotsInSpace[i]).x[1]), 2) + pow( ((p2.ShotsInSpace[i]).y[1]), 2) ) ,-1));
                (p2.ShotsInSpace[i]).SetForces(0.,0.);
                
                (p2.ShotsInSpace[i]).draw(window);
                
                for (int l = 0 ; l < (PlanetsInSpace).size(); l++) if (!PlanetsInSpace[l].isDead()) (p2.ShotsInSpace[i]).externalForce(PlanetsInSpace[l]);

                //If the shot is no longer permited then erase
                if( (*( (p2.ShotsInSpace).begin()+i )).LivingTime()) {
                    p2.EndFire();
                    (p2.ShotsInSpace).erase((p2.ShotsInSpace).begin()+ i);
                }
                else if ( !PlanetsInSpace[0].isDead() && ( (p2.ShotsInSpace)[i].shape.getGlobalBounds().intersects( PlanetsInSpace[0].shape.getGlobalBounds() ) )){
                    PlanetsInSpace[0].life -= 10;
                    p2.EndFire();
                    (p2.ShotsInSpace).erase((p2.ShotsInSpace).begin()+ i);
    
                }
                else if ( !PlanetsInSpace[1].isDead() && ( (p2.ShotsInSpace)[i].shape.getGlobalBounds().intersects( PlanetsInSpace[1].shape.getGlobalBounds() ) )){
                    PlanetsInSpace[1].life -= 10;
                    p2.EndFire();
                    (p2.ShotsInSpace).erase((p2.ShotsInSpace).begin()+ i);
                }

            }
            
            //Update position of all the elements related to the player p2
            p2.UpdatePosition();
            
            //Check control
            p2.GetAll();

        //Display effect of the planets (Penser a rajouter une animation XD)
            for (int i = 0 ; i < (PlanetsInSpace).size(); i++ ){
                
                if (75 < PlanetsInSpace[i].life && PlanetsInSpace[i].life <= 100)
                {
                    PlanetsInSpace[i].texture.loadFromFile("spaceMeteors_001.png");
                    PlanetsInSpace[i].texture.setSmooth(true);
                    PlanetsInSpace[i].shape.setTexture(PlanetsInSpace[i].texture);
                }
                else if ( 50 < PlanetsInSpace[i].life && PlanetsInSpace[i].life <= 75)
                {
                    PlanetsInSpace[i].texture.loadFromFile("spaceMeteors_002.png");
                    PlanetsInSpace[i].texture.setSmooth(true);
                    PlanetsInSpace[i].shape.setTexture(PlanetsInSpace[i].texture);
                }
                else if (25 < PlanetsInSpace[i].life && PlanetsInSpace[i].life <= 50)
                {
                    PlanetsInSpace[i].texture.loadFromFile("spaceMeteors_003.png");
                    PlanetsInSpace[i].texture.setSmooth(true);
                    PlanetsInSpace[i].shape.setTexture(PlanetsInSpace[i].texture);
                }
                else if (PlanetsInSpace[i].life <= 25)
                {
                    PlanetsInSpace[i].texture.loadFromFile("spaceMeteors_004.png");
                    PlanetsInSpace[i].texture.setSmooth(true);
                    PlanetsInSpace[i].shape.setTexture(PlanetsInSpace[i].texture);
                }

                if (!PlanetsInSpace[i].isDead())
                {
                    window.draw(PlanetsInSpace[i].shape);
                }
                    
            }

        window.draw(p2.shape);
        window.draw(p.shape);
        window.display();

    }

    p.~ship();
    p2.~ship2();
    mars.~planet();
    moon.~planet();
    return 0;
}

