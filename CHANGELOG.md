## v0.2.2 (2023-02-06)

### Fix

- **hangar**: add finner to hangar namespace import

## v0.2.1 (2023-02-06)

### Fix

- **Component**: overload copy and deepcopy dunders
- **utilities.py**: use component name tags where possible
- **README.md**: broken link to x43a example docs

## v0.2.0 (2023-02-02)

### Feat

- **polygon.py**: updated polygon to standard component type
- **SensitivityStudy**: allow passing nominal stl prefix
- **Vehicle**: improved verbosity of to_stl and prefix control
- **Vehicle**: component name tags will be used when writing to stl
- **Component**: improved repr
- **SensitivityStudy**: allow passing outdir when saving to csv
- **hangar**: expose all vehicles in hangar to package
- **Vehicle**: allow specifying vehicle transformations prior to generate()
- **CompositeComponent**: added new component type
- **append_sensitivities_to_tri**: added control of matching tolerances
- **append_sensitivities_to_tri**: write csv of combined sensitivity data
- **append_sensitivities_to_tri**: capability for multiple parameters
- **Vehicle**: allow chained transformations of components
- **common.py**: added circle patch function
- ability to cluster stl meshing

### Fix

- **hangar**: updated vehicles to migrate to general components
- **hangar**: call super()
- **hangar**: inherit Generator instead of AbstractGenerator
- **hangar**: attributes for vehicle parameters
- **append_sensitivities_to_tri**: do not write index to csv

### Refactor

- **Fuselage**: split fuselage component into more general revolved and swept components
- **scripts**: deleted redundant scripts directory

## v0.1.0 (2023-01-24)

### Feat

- **Generator**: implemented base generator class
- **Vehicle**: implemented analyse method
- **utilities**: reimplemented interial properties utility
- **SensitivityStudy**: reimplemented
- **Vehicle**: added enumerated_components attribute
- **Component**: added Component to hypervehicle.components namespace
- **AbstractGenerator**: added abstract generator class
- **Vehicle**: added banner
- **hifire8.py**: translated hifire8 model
- **hifire4.py**: translated hifire4 model
- **Component**: improved component curvature implementation
- **x43.py**: translated x43 model
- **rocket.py**: translated generic rocket model
- **falcon9.py**: translated falcon 9 model
- **htv.py**: translated HTV model
- **Vehicle**: implemented control of component reflections
- **Component**: implemented mesh analysis method
- **Component**: implemented stl_resolution arg
- **Vehicle**: added verbosity control
- **OgiveNose**: implemented ogive nose as common component
- **common.py**: added uniform thickness function generator
- **Component**: check if patches have been generated when creating surfaces
- **Vehicle**: implemented to_stl method with component name iteration
- **Vehicle**: added component counts to vehicle repr
- refex vehicle working
- **geometry.py**: added Arc to namespace
- added legacy methods to components to transition
- **Vehicle**: started new implementation
- **transformations.py**: added standard transformations module
- **Component**: implemented reflect method
- **Component**: added methods for component processing
- **constants**: added constants component definitions
- updated fuselage and fin components
- **wing.py**: updated Wing component
- **component.py**: added abstract class for components
- **utils.py**: added csv to delaunay formatterr
- implemented swept fuselage component
- option to specify sweep axis
- **utils**: added surface perimeter utility class
- added ReFEX to hangar
- allow using default LE func for fin
- added falcon9 to hangar
- component-specific stl resolutions
- ability to specify revolve line for fuselage
- ability to stack fuse components
- added revolved surface class
- added htv to hangar
- added fuselage offset function
- updated docs
- added fin and fuselage constructor methods
- improved sensitivity naming in tri files
- improved formatting
- only write nominal geometry to file
- added distinction between building and writing STL to file
- added utility to add sensitivity data to .i.tri file
- first pass of geometric differentiation complete
- added close wing option
- added a template for using the polygon_formation module.
- Added new module polygon_formation.py and new component polygon.py
- added cube and sphere patch functions
- added tests badge

### Fix

- **components**: removed stale curvature function arguments
- **Component**: fixed swept fuselage surface gen
- **SensitivitiyStudy**: updated to new structure
- **d21.py**: partially translated d21
- **Wing**: round bisect result for t_B1
- **uniform_thickness_function**: x and y coordinates
- **Vehicle**: transform using transformations from argument
- apply rounding to t_B1 bisect
- **Component**: added temporary means of component reflection
- **Fin**: save fin patches dict
- deleted outdated template file
- **Vehicle**: removed stale Vehicle methods
- flip end face for swept fuselage components
- match stl resolution for swept surfaces
- handle negative cross section locations
- **fuselage.py**: fixed return type hinting
- added offset func arg for fin components
- fuselage curvature implementation
- apply rudder angle before axial rotations
- rotate fuselage for cart3d
- verbosity output
- NumberOfComponents specification
- variable assignment
- fin negative volume error
- old import from idmoc
- tests link
- imports

### Refactor

- **Vehicle**: changed ordering in transformations tuple
- **geometry.pu**: added Spline to geometry namespace
- merged component legacy methods into init
- split utils.py into geometry.py and utilities.py
- updated all imports to use gdtk namespace

### Perf

- **SweptPatch**: significantly improved time to __call__ swept patch
