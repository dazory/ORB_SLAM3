# ORB-SLAM3

## Week3

If (IMU measured && enough acceleration): New map created with 513 points.

1. `Tracking::Track()`
  
    If we have an initial estimation of the camera pose and matching. Track the local map.
    
    1. `Tracking::TrackLocalMap()`

        1. UpdateLocalMap

            1. `mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);`
            2. UpdateLocalKeyFrames: `mvpLocalMapPoints`에 관련있는 keyframes을 모두 push.
            3. UpdateLocalPoints: `mvpLocalMapPoints`에 관련있는 map points를 모두 push.

        2. SearchLocalPoints

            1. 현재 프레임의 모든 `mvpMapPoints`에 대해, (1) increaseVisible() (2) set  `LastFrameSeen=mCurrentFrame.mnId; mbTrackInView=false; mbTrackInViewR=false;`
                * `mbTrackInView=false`: 아래에서 `mmProjectPoints`에 projection하지 않게됨.

            2. 현재 프레임의 모든 `mvpLocalMapPoints`에 대해:

                * Frustum 내에 존재하는 map point이면: increaseVisible() && `nToMatch++`
                * `mbTrackInView=true`: `mCurrentFrame.mmProjectPoints`에 `(mTrackProjX, mTrackProjY)`을 할당.

            3. match해야하는 map points가 존재하면 (`nToMatch>0`)

                * ORBmatcher(`nnratio=0.8`)를 생성하고, `mvpLocalMapPoints`을 `th`으로 `mCurrentFrame`에 projection.

        * If (TrackLocalMap success): `bOK=true`

    2. Save frame & Reset: `mState = OK;`

    3. (If `bOK=true`) Update model & Clean, Delete

        1. Update motion model => `mVelocity`

        2. Clean VO matches: For all map points in current frame (pMP), set mvbOutliers=false & mvpMapPoints=NULL if `observation < 1`.

        3. Delete temporal MapPoints.

    4. (If `bOK=true`) NeedNewKeyFrame()

        * (If `IMU` && `IMU.initialized` && timestemp interval <= 0.25): return false // => Not CreateNewKeyFrame; (초반에 여기를 3번정도 돎)
        * (If `IMU` && `IMU.initialized` && timestemp interval >= 0.25): return true // => CreateNewKeyFrame; (이후에 여기로 들어옴)

    5. (If `NeedNewKeyFrame` && bOK) CreateNewKeyFrame()

        * (If `Stereo` && ) Update pose matrices: `mCurrentFrame.UpdatePoseMatrices();`

            * `vector<pair<float,int>> vDepthIdx.push_back(z, i)` for `z`: mCurrentFrame.mvDepth; `i`: index of keypoints;
            * (If `!vDepthIdx.empty`) For all `vDepthIdx` (`j`): 

                For all index of mvDepth (`i`):
                1. (If mvpMapPoints[i].empty) mvpMapPoints[i] = NULL; `bCreateNew=true`
                2. (If `bCreateNew`) 
                    1. x3D = mCurrentFrame.UnprojectStereo(i)
                    2. pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap())
                    3. pNewMP->AddObservation(pKF, i); pKF->AddMapPoint(pNewMP, i);
                    4. pNewMP-ComputeDistictiveDescriptors(); pNewMP->UpdateNormalAndDepth();
                    5. mpAtlas->AddMapPoint(pNewMP)
                    6. mCurrentFrame.mvpMapPoints[i]=pNewMP
                3. nPoints++
                4. (If `z>mThDepth` && `nPoints>maxPoint`) break;
        * mpLocalMapper->InsertKeyFrame(pKF)
        * mpLocalMapper->SetNotStop(false)
        * mnLastKeyFrameId = mCurrentFrame.mnId
        * mpLastKeyFrame = pKF
    
    6. (If `mState=LOST`) Reset when LOST
        
        * (If `pCurrentMap->KeyFramesInMap()<=5` or `IMU && !IMU.initialized`) mpSystem->ResetActiveMap(); return;
        * CreateMapInAtlas()

    7.  mLastFrame = Frame(mCurrentFrame)
    8. (If `mState==OK` or `mState==RECENTLY_LOST`) Save frame pose information

        * (If `!mCurrentFrame.mTcw.empty()`) store mpReferenceKF's information
            * `mlRelativeFramePoses` <- `mCurrentFrame.mpReferenceKF->GetPoseInverse()`
            * `mlpReferences` <- `mCurrentFrame.mpReferenceKF`
            * `mlFrameTimes` <- `mCurrentFrame.mTimeStamp`
            * `mlbLost` <- `mState==LOST`
        * (else: If tracking is lost) store privious mpReferenceKF's information"
            * `mlRelativeFramePoses` <- `mlRelativeFramePoses.back()`
            * `mlpReferences` <- `mlpReferences.back()`
            * `mlFrameTimes` <- `mlFrameTimes.back()`
            * `mlbLost` <- `mState==LOST`

    9. 



  