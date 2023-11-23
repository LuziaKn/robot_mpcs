# Using forcespro in a docker

Resquest a floating license from Embotech.

To generate a solver with floating license, you need to set the following options in your solver:

```
options.license.use_floating_license = 1
```

You can generate the solver outside of the docker and it is possible to generate it with another license attached to the same account.
Go inside the dowloaded folder and run

```
./forcespro_floating_licenses_proxy
```

If the exit flag is not -100 the floating solver is active.