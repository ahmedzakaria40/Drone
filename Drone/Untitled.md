### Week 1 — Physics engine in C

Implement:

```
typedef struct {
    double x;
    double y;
    double vx;
    double vy;
    double theta;
    double omega;
} DroneState;
```

Deliverables:

- `DroneState`
- Drone parameters
- Physics derivative function
- Euler integrator
- Command-line simulation
- First trajectory plot