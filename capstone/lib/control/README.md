# capstone

Controller designed for the completion of the final Capstone Mars mission project of Prof. Schaub's Spacecraft Dynamics &amp; Control course (University of Colorado Boulder).

Control implemented for science satellite craft in Low Mars Orbit (LMO), in three mission phases:
 - 'Science' phase: craft must orient itself so sensors are pointing along the nadir direction.
 - Charging phase: craft orients its solar panels towards the sun in order to recharge batterie(s) on board.
 - Communication phase: craft must orient itself such that its communnication antennae points towards mothership in Geostationary Mars Orbit (GMO).
 
Simple PD controller is implemented on attitude and rate errors. Attitudes described by Modified Rodriguez Parameters, switched to avoid singularity at +/- 360 degs.

To use this control, substitute this repository for the control directory in [propagate](https://github.com/kierandwn/propagate).
