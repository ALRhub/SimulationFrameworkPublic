# New DeepMind MuJoCo bindings

With this pull request, we are adding the new MuJoCo bindings to the ALR Simulation Framework.
This is an Open Beta release, so please consider this as a preview. You are free
to use it, but please let us know if you have any comments or suggestions.

**Report any issues to the ALR-Sim GitHub issue tracker!**

You can access the new MuJoCo bindings by using the following lines of code:

```python
    from alr_sim.sims.SimFactory import SimRepository
    sim_factory = SimRepository.get_factory("mj_beta")
```
