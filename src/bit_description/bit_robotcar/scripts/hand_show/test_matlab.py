import time
import matlab.engine
eng = matlab.engine.start_matlab()
ret=eng.count([[1,2,3],[4,5,6]])
type(ret)
eng.quit()