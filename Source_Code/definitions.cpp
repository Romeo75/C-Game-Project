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

//DEBUT DU COMPTEUR (Chronomètre qui compte en ms)
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
                   
        /// TEXTURES ....

        Sprite shape;
        Texture texture;

        
        double thrust,phy; //phy est l'angle de rotation

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
            if (x[0] > windowSizeX) x[0]=0; if (x[0] < 0) x[0]=windowSizeX;
            if (y[0] > windowSizeY) y[0]=0; if (y[0] < 0) y[0]=windowSizeY;
            
            //Deffines Speed limits
            if ( x[1] > abs(MaxSpeed) ){x[1]=MaxSpeed;}
            if ( y[1] > abs(MaxSpeed) ){y[1]=MaxSpeed;}
            if ( x[1] < -abs(MaxSpeed) ){x[1]=-MaxSpeed;}
            if ( y[1] < -abs(MaxSpeed) ){y[1]=-MaxSpeed;}
        }
        double * GetVectorX(){ return x;}
        double * GetVectorY(){ return y;}

        double GetDirectionX() { return cos( phy*0.017453f );}
        double GetDirectionY() { return sin( phy*0.017453f );}

        void SetPosition(double xPos,double yPos){
            this->x[0] = xPos;
            this->y[0] = yPos;
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

        double ttl; // TEMPS DE VIE DU MISSILE

        double createdTime;

	public:

        void UpdatePosition(){
            
            cout<< "\n................................................"<<endl;
            rk4(3, 0., x, 1e-1);
            rk4(3, 0., y, 1e-1);
            ApplyLimits();
            shape.setPosition(x[0],y[0]);
            shape.setRotation(phy-90);

        }    

        void externalForce(space_object& a){
            
            //cout << "distance de la planete: " << distance(a);

            x[2] += a.gravity * (a.x[0]-x[0])*pow(distance(a),-3);
            y[2] += a.gravity * (a.y[0]-y[0])*pow(distance(a),-3);

            
        }

        // SUPRESSION DU MISSILE SI TEMPS DE VIE EXPIRE
        bool LivingTime(){
            return (GetTickCount() - createdTime >= ttl);
        }

        //Shot Consructor
        Shot(string name, double gravity, int life, Color color, string picture, double sizeX, double sizeY, double ttl, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[],double phy):space_object(name, gravity, life, color, picture, sizeX, sizeY, windowSizeX, windowSizeY, MaxSpeed , x, y,phy){
            this-> ttl = ttl;
            this-> createdTime = GetTickCount();
        }

        // SUPPRIME LE MISSILE
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
                 <<  "Life: " << life << endl
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

                    double x[]={GetVectorX()[0], GetDirectionX()*5e2, 0};
                    double y[]={GetVectorY()[0], GetDirectionY()*5e2, 0};
                    
                    // Makes new Shot
                    Shot shot( "Boom... ", 1., 3, Color::Black, "res/spaceMissil.png", 20, 35, 3e3, GetWindowSizeX(), GetWindowSizeY(), 2e2, x, y, phy + 180);
                    
                    //Stores it in a vector
                    (this->ShotsInSpace).push_back(shot);
                    
                    // Last Shot fired now!!!!
                    lastShotTime = GetTickCount();
                    shotsUsed++;

                }
            }
            
        }

    
    // Stop firing a bullet (called back when a Bullet object is destroyed)
    void ship::EndFire(){
        shotsUsed = max(shotsUsed - 1, 0);
    }

    // Reset cooldown (used when fire key is released)
    void ship::ResetShotCooldown(){
        lastShotTime = 0;
    }


    void ship::GetInput(int sensibility){

            if ( Keyboard::isKeyPressed(sf::Keyboard::Left) )  {        this->phy   += -sensibility;
                if ( Keyboard::isKeyPressed(sf::Keyboard::Up) ){        this->thrust += sensibility;
                    if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                    else firing = false;
                }
                else if ( Keyboard::isKeyPressed(sf::Keyboard::Down) ){ this->thrust += -sensibility;
                    if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                    else firing = false;
                }
                else if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                else firing = false;
            }

            else if ( Keyboard::isKeyPressed(sf::Keyboard::Right) ) {   this->phy   += +sensibility;
                if ( Keyboard::isKeyPressed(sf::Keyboard::Up) ){         this->thrust += sensibility;
                    if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                    else firing = false;
                }
                else if ( Keyboard::isKeyPressed(sf::Keyboard::Down) ){  this->thrust += -sensibility;
                    if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                    else firing = false;
                }
                else if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                else firing = false;
            }
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Up) ){        this->thrust += sensibility;
                if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                else firing = false;
            }
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Down) ){      this->thrust += -sensibility;
                if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
                else firing = false;
            }

            else if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
            else {

                thrust = 0.;
                this->GetVectorX()[2] = 0.;
                this->GetVectorY()[2] = 0.;
                ResetShotCooldown();
                firing = false;
            
            }

            GetVectorX()[2] = thrust*GetDirectionX();
            GetVectorY()[2] = thrust*GetDirectionY();

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

    if ( Keyboard::isKeyPressed(sf::Keyboard::A) )  {        this->phy   += -sensibility;
        if ( Keyboard::isKeyPressed(sf::Keyboard::W) ){        this->thrust += sensibility;
            if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
            else firing = false;
        }
        else if ( Keyboard::isKeyPressed(sf::Keyboard::S) ){ this->thrust += -sensibility;
            if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
            else firing = false;
        }
        else if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
        else firing = false;
    }

    else if ( Keyboard::isKeyPressed(sf::Keyboard::D) ) {   this->phy   += +sensibility;
        if ( Keyboard::isKeyPressed(sf::Keyboard::W) ){         this->thrust += sensibility;
            if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
            else firing = false;
        }
        else if ( Keyboard::isKeyPressed(sf::Keyboard::S) ){  this->thrust += -sensibility;
            if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
            else firing = false;
        }
        else if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
        else firing = false;
    }
    else if ( Keyboard::isKeyPressed(sf::Keyboard::W) ){        this->thrust += sensibility;
        if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
        else firing = false;
    }
    else if ( Keyboard::isKeyPressed(sf::Keyboard::S) ){      this->thrust += -sensibility;
        if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
        else firing = false;
    }

    else if ( Keyboard::isKeyPressed(sf::Keyboard::F) ) firing = true;
    else
    {
        thrust = 0.;
        this->GetVectorX()[2] = 0.;
        this->GetVectorY()[2] = 0.;
        ResetShotCooldown();
        firing = false;
    }

    GetVectorX()[2] = thrust*GetDirectionX();
    GetVectorY()[2] = thrust*GetDirectionY();

}

//Class that defines the planets
class planet: public space_object{

    private:


    public:
        
        void UpdatePosition(){
                
                cout<<"\n......";
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

    /// Menu ....

        system("clear");
        int windowSizeX = 1200, windowSizeY = 700;
        
        string name1,name2;

        cout << "\nInserez le nom du joueur N°1: ";cin >> name1;
        cout << "\nInserez le nom du joueur N°2: ";cin >> name2;

        cout << "\nInserez la taille horizontale de la fenetre de jeu: ";   cin >> windowSizeX;
        cout << "\nInserez la taille verticale de la fenetre de jeu: ";     cin >> windowSizeY;
    
    /// PROPRIETES DU FOND ....
        
        
        RenderWindow window(VideoMode(windowSizeX, windowSizeY), "Spacecraft Movement");
        window.setFramerateLimit(40);
        Texture t1;
        t1.loadFromFile("res/blue.png");
        t1.setRepeated(true);
        Sprite sFond(t1,IntRect(0,0,windowSizeX,windowSizeY));

    /// PROPRIETES DES OBJETS ....
    
        double textX = 0,textY = 0;
        double sizeX = 100, sizeY = 94;     // dimensions of the ship
        double sizePX = 215, sizePY = 211;  // dimensions of the planet
        
        double gravity = 1e7;
        int life = 100;
        
        double x[]={windowSizeX/4.,0.,0.},y[]={windowSizeY*0.5,0.,0.}; // position initiale du veseau x[0],y[0], vitesse x[1], y[1] et acceleration x[3], x
                                                            // x[2] et y[2] Intensité des forces subies par le cercle exprimees dans la base canonique.
        int vmax = 100, maxShots = 10;
        double phy = 0;

    /// Objects ....

        ship p(name1, gravity, life, Color::Green, "res/spaceShip_01.png", sizeX, sizeY, maxShots, windowSizeX, windowSizeY, vmax, x, y, phy);

        vector<planet> PlanetsInSpace;
        planet mars( "Mars", gravity, life, Color::Blue, "res/meteor.png", sizePX, sizePY, windowSizeX, windowSizeY, vmax, x, y, phy );
        
        x[0]=windowSizeX*3/4.;y[0]=windowSizeY*0.5;
        sizeX = 106, sizeY = 80;
        ship2 p2(name2, gravity, life, Color::Green, "res/spaceShip_02.png", sizeX, sizeY, maxShots, windowSizeX, windowSizeY, vmax, x, y, phy);
        planet moon("Moon", gravity, life, Color::Blue, "res/meteor.png", sizePX, sizePY, windowSizeX, windowSizeY, vmax, x, y, phy );

        PlanetsInSpace.push_back(mars);
        PlanetsInSpace.push_back(moon);
        
        Font mainFont;
        mainFont.loadFromFile("res/kenvector_future.ttf");
        Font thinFont;
        thinFont.loadFromFile("res/kenvector_future_thin.ttf");

        Text PlayerVictory;
        PlayerVictory.setFont(mainFont);
        PlayerVictory.setCharacterSize(20);
        PlayerVictory.setColor( Color::White );
        PlayerVictory.setPosition(windowSizeX*0.5,windowSizeY*0.5);

        Text playerData;
        playerData.setFont(thinFont);
        playerData.setCharacterSize(20);
        playerData.setFillColor(Color::White);
        playerData.setPosition(windowSizeX*(p.GetName().length()/(double)windowSizeX), 0);
        
        Text player2Data;
        player2Data.setFont(thinFont);
        player2Data.setCharacterSize(20);
        player2Data.setFillColor(Color::White);
        player2Data.setPosition(windowSizeX*(1 - 5*(pow(p2.GetName().length(),2)/(double)windowSizeX)), 0);

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
                //PlanetsInSpace[i].GetAll();
                PlanetsInSpace[i].UpdatePosition();
                //PlanetsInSpace[i].SetForces(PlanetsInSpace[0].RandPosition( 10 ), PlanetsInSpace[0].RandPosition( 10 ));
            }
        /* 
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
        */


        //Player 1 Conditions    
        if ( !p.isDead() ){
            //Get the input from the arrows in the keyboard for player1
            p.GetInput(3);
            if ( p.firing ){
                
                cout<< endl << p.GetName() + "  Fireeee!!!!!!!";
                p.Fire();
                //p.ShotsInSpace[0].GetAll();

            }
        
            //Sequence that updates all of the shots in space
            for (int i = 0 ; i < (p.ShotsInSpace).size(); i++){
                
                (p.ShotsInSpace[i]).UpdatePosition();    
                (p.ShotsInSpace[i]).texture.loadFromFile("res/spaceMissil.png");
                (p.ShotsInSpace[i]).shape.setTexture((p.ShotsInSpace[i]).texture);
              //(p.ShotsInSpace[i]).GetAll();
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
                else if (  ( (p.ShotsInSpace)[i].shape.getGlobalBounds().intersects( p2.shape.getGlobalBounds() ) )) {
                    p2.life -= 10;
                    (p.ShotsInSpace).erase((p.ShotsInSpace).begin()+ i);
                    }
            }
            
            //Update position of all the elements related to the player p
            p.UpdatePosition();
            
            //Check control
            //p.GetAll();
        }

        //Player 2 Conditions
        if ( !p2.isDead() ){
            //Get the input from the arrows in the keyboard for player1
            p2.GetInput(3);
            if ( p2.firing ){
                
                cout<< endl << p2.GetName() + " Fireeee!!!!!!!";
                p2.Fire();
                //p2.ShotsInSpace[0].GetAll();

            }
        
            //Sequence that updates all of the shots in space
            for (int i = 0 ; i < (p2.ShotsInSpace).size(); i++){
                
                (p2.ShotsInSpace[i]).UpdatePosition();    
                (p2.ShotsInSpace[i]).texture.loadFromFile("spaceMissil.png");
                (p2.ShotsInSpace[i]).shape.setTexture((p2.ShotsInSpace[i]).texture);
                //(p2.ShotsInSpace[i]).GetAll();
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
                else if ( !p.isDead()  && ( (p2.ShotsInSpace)[i].shape.getGlobalBounds().intersects( p.shape.getGlobalBounds() ) )) {
                    p.life -= 10;
                    (p2.ShotsInSpace).erase((p2.ShotsInSpace).begin()+ i);
                    }

            }
            
            //Update position of all the elements related to the player p2
            p2.UpdatePosition();
            
            //Check control
            //p2.GetAll();
        }
        //Affichage des planetes (petite animation XD)
            for (int i = 0 ; i < (PlanetsInSpace).size(); i++ ){
                
                if (75 < PlanetsInSpace[i].life && PlanetsInSpace[i].life <= 100)
                {
                    PlanetsInSpace[i].texture.loadFromFile("res/spaceMeteors_001.png");
                    PlanetsInSpace[i].texture.setSmooth(true);
                    PlanetsInSpace[i].shape.setTexture(PlanetsInSpace[i].texture);
                }
                else if ( 50 < PlanetsInSpace[i].life && PlanetsInSpace[i].life <= 75)
                {
                    PlanetsInSpace[i].texture.loadFromFile("res/spaceMeteors_002.png");
                    PlanetsInSpace[i].texture.setSmooth(true);
                    PlanetsInSpace[i].shape.setTexture(PlanetsInSpace[i].texture);
                }
                else if (25 < PlanetsInSpace[i].life && PlanetsInSpace[i].life <= 50)
                {
                    PlanetsInSpace[i].texture.loadFromFile("res/spaceMeteors_003.png");
                    PlanetsInSpace[i].texture.setSmooth(true);
                    PlanetsInSpace[i].shape.setTexture(PlanetsInSpace[i].texture);
                }
                else if (PlanetsInSpace[i].life <= 25)
                {
                    PlanetsInSpace[i].texture.loadFromFile("res/spaceMeteors_004.png");
                    PlanetsInSpace[i].texture.setSmooth(true);
                    PlanetsInSpace[i].shape.setTexture(PlanetsInSpace[i].texture);
                }

                if (!PlanetsInSpace[i].isDead())
                {
                    window.draw(PlanetsInSpace[i].shape);
                }
                    
            }

            if ( !p2.isDead() ) window.draw(p2.shape);
            else {
                cout << "\n" << p.GetName() << "  Wiiiinnnnnsssss!!!!";
                PlayerVictory.setString(p.GetName()+"  Wins!!!!");
                
            }
            if ( !p.isDead() ) window.draw(p.shape);
            else { 
                cout << "\n" << p2.GetName() << "  Wiiiinnnnnsssss!!!!";
                PlayerVictory.setString(p2.GetName()+"  Wins!!!!");
            }
        //Donees du joueur 1
        playerData.setString(p.GetName() + "\nHP:   " + to_string(p.life));
        //Donnees du joueur 2
        player2Data.setString(p2.GetName() + "\nHP:   " + to_string(p2.life));

        window.draw(playerData);
        window.draw(player2Data);
        window.draw(PlayerVictory);
        window.display();

    }

    return 0;
}

