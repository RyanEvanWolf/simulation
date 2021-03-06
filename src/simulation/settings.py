

def MotionCategorySettings():
    Settings={}
    Settings["Fast"]={}
    Settings["Medium"]={}
    Settings["Slow"]={}
    Settings["Fast"]["TranslationMean"]=0.066
    Settings["Fast"]["RotationMean"]=30
    Settings["Fast"]["TranslationNoise"]=0.1*Settings["Fast"]["TranslationMean"] ##meters
    Settings["Fast"]["RotationNoise"]=8    ##degrees

    Settings["Medium"]["TranslationMean"]=0.044
    Settings["Medium"]["RotationMean"]=8
    Settings["Medium"]["TranslationNoise"]=0.2*Settings["Medium"]["TranslationMean"] ##meters
    Settings["Medium"]["RotationNoise"]=2        ##degrees

    Settings["Slow"]["TranslationMean"]=0.022
    Settings["Slow"]["RotationMean"]=10
    Settings["Slow"]["TranslationNoise"]=0.5*Settings["Slow"]["TranslationMean"] ##meters
    Settings["Slow"]["RotationNoise"]=3        ##degrees
    return Settings


def getSimulatedLandmarkSettings():
    Settings={}
    Settings["Xdepth"]=5
    Settings["Ydepth"]=5
    Settings["Zdepth"]=5
    Settings["HeightMaximum"]=0.5
    Settings["MinimumOutlier"]=4.0 #pixels
    Settings["OutlierLevels"]=[0.05,0.12,0.2]
    Settings["GaussianNoise"]=[0.25,0.75,1.25]
    Settings["operatingCurves"]=[0.1,0.4,0.7,1.0]
    return Settings 
