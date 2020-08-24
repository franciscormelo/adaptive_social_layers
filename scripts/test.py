
import matlab.engine
eng = matlab.engine.start_matlab()
eng.cd(r'/home/flash/catkin_ws/src/adaptive_social_layers/scripts', nargout=0)

mdl = 8000
stride = 50
persons = matlab.double([[1,1,3,4],[2,2,4,4], [3,300,4000,5]])
ret= eng.gcff(mdl,stride, persons)
print(ret)
eng.quit()
