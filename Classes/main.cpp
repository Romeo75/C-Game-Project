#include <cstdlib>
#include <iostream>
#include<math.h>
#include <SFML/Graphics.hpp>

//#include "ResourcePath.hpp"

using namespace std;
using namespace sf;

const int W = 1200;         // largeur de la fenetre
const int H = 800;          // hauteur de la fenetre
const int rayon = 60;       // vaisseau est un cercle

float DEGTORAD = 0.017453f;

int main ()
{

    RenderWindow app(VideoMode(W,H), "SPAAACE");
    app.setFramerateLimit(60);
    
    /// TEXTURES \\\
    
    Texture t1,t2;
    t1.loadFromFile("blue.png");
    t1.setRepeated(true);
    t2.loadFromFile("spaceShip.png");
    
    /// SPRITES \\\
    
    Sprite sFond(t1,IntRect(0,0,W+500,H)),sPlayer(t2);
    sPlayer.setTextureRect(IntRect(0,0,100,94));
    sPlayer.setOrigin(50,47);
    
    float x=300, y=300;
    float dx=0,dy=0,angle=0;
    bool thrust;
    
    
    while (app.isOpen())
    {
        Event event;
        while (app.pollEvent(event))
        {
            if (event.type == Event::Closed)
                app.close();
        }
        
        if (Keyboard::isKeyPressed(Keyboard::Right)) angle +=3;
        if (Keyboard::isKeyPressed(Keyboard::Left)) angle -=3;
        if (Keyboard::isKeyPressed(Keyboard::Up)) thrust=true;
        else thrust=false;
        
        /// MOUVEMENT DU VAISSEAU \\\
        
        if (thrust)
        {
            dx+=cos(angle*DEGTORAD)*0.2;
            dy+=sin(angle*DEGTORAD)*0.2;
        }
        else
        {
            dx*=0.99;
            dy*=0.99;
        }
        
        int Vmax=15;
        float V = sqrt(dx*dx+dy*dy);
        if(V>Vmax)
        {
            dx *= Vmax/V;
            dy *= Vmax/V;
        }
        
        x+=dx;
        y+=dy;
        
        if (x > W) x=0; if (x < 0) x=W;
        if (y > H) y=0; if (y < 0) y=H;
        
        sPlayer.setPosition(x, y);
        sPlayer.setRotation(angle-90);
        
        
        /// AFFICHAGE \\\
        
        app.clear();
        app.draw(sFond);
        app.draw(sPlayer);
        app.display();
    }
    
    return 0;
}
