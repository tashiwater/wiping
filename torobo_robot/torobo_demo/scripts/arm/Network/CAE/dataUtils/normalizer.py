import numpy as np

def angle2data(angle, angRange, dataRange):
    # angle ... ndarray. [5,]
    r_data = np.zeros(angle.shape)
    for i in range(angRange.shape[0]):
        r_data[i] = angle[i]
        r_data[i] = ( r_data[i] - angRange[i,0] ) / abs( angRange[i,1] - angRange[i,0] ) \
                    * abs( dataRange[1] - dataRange[0] )
        r_data[i] = r_data[i] + dataRange[0]
    return r_data


def data2angle(data, angRange, dataRange):
    # data ... ndarray. [5,]
    r_angle = np.zeros(data.shape)
    for i in range(angRange.shape[0]):
        r_angle[i] = data[i]
        r_angle[i] = ( r_angle[i] - dataRange[0] ) / abs( dataRange[1] - dataRange[0] ) \
                     * abs( angRange[i,1] - angRange[i,0] )
        r_angle[i] = r_angle[i] + angRange[i,0]
    return r_angle

def data2angle_seq(data, angRange, dataRange):
    # data ... ndarray. [T, 5]
    r_angle = np.zeros(data.shape)
    for t in range(data.shape[0]):
        for i in range(angRange.shape[0]):
            r_angle[t,i] = data[t,i]
            r_angle[t,i] = ( r_angle[t,i] - dataRange[0] ) / abs( dataRange[1] - dataRange[0] ) \
                         * abs( angRange[i,1] - angRange[i,0] )
            r_angle[t,i] = r_angle[t,i] + angRange[i,0]
    return r_angle
