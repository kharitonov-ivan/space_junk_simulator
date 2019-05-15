### Добавление новых сил
Чтобы добавить свою силу, нужно:

1. Добавить её код в проект, например:
https://github.com/neer201/space_junk_simulator/blob/dev/src/gpu/gpu_solver.cu#L11

2. Добавить её вызов в метод GetAccel, например:
https://github.com/neer201/space_junk_simulator/blob/dev/src/gpu/gpu_solver.cu#L209

3. Добавить необходимые опции в структуру. Они будут переданы в функцию:
https://github.com/neer201/space_junk_simulator/blob/dev/src/gpu/gpu_solver.cu#L7

4. Добавить названием функции в BuildOptions, например:
https://github.com/neer201/space_junk_simulator/blob/dev/src/gpu/gpu_solver.cu#L329

5. Пересобрать бинарник, обновить конфиг.
