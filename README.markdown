Introduction
============

RealPaver is used for nonlinear constraint solving & rigorous global optimization.

[Home Page](http://pagesperso.lina.univ-nantes.fr/info/perso/permanents/granvil/realpaver/)

We're using it for the simulation of the drive cycle of a hydraulic vehicle and want to say things like...

    acceleration[t] = (velocity[t] - velocity[t-1]) * dt
    
Unfortunately, there is not a construct in RealPaver for defining constraints over sets. So, for the previous example, a line would have to be written for every time step t.

That's where this code comes in. It let's you say 

    quick_constraint("acceleration[t] = (velocity[t] - velocity[t-1]) * dt for t in times")

And the corresponding RealPaver model will be generated.

There are a couple of other useful features that come from using a powerful language like Python instead of simply a constraint programming language.

Implementation
==============

This is all implemented as a YACC grammar using ply. As a result the implementation is compact. It could be even more so once our lab commits to RealPaver and can justify spending time on this software piece.