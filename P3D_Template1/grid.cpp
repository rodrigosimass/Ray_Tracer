#include "grid.h"
#include "maths.h"

Mail::Mail()
{
    ray_id = -1;
    hit = false;
}

Mailbox::Mailbox(void)
{
}

Mailbox::Mailbox(int nObj)
{
    box = (Mail *)malloc(nObj * sizeof(Mail));
    /* for (int i = 0; i < nObj; i++)
    {
        box.push_back(Mail());
    } */
}

bool Mailbox::check(int obj_id, int r_id, float &dist, bool &h)
{
    if (box[obj_id].ray_id != r_id) //MISS
        return false;               //signifies that there was a miss
    else
    {
        dist = box[obj_id].t;
        h = box[obj_id].hit;
        return true; //HIT
    }
}

void Mailbox::update(int obj_id, int r_id, float dist, bool h)
{
    box[obj_id].ray_id = r_id;
    box[obj_id].t = dist;
    box[obj_id].hit = h;
}

Grid::Grid(vector<Object *> objs, int num_obj)
{
    objects = objs;
    N = num_obj;
    m = GRID_M;
}

Grid::Grid(void)
{
    N = 0;
}

Object *Grid::getObject(unsigned int index)
{
    if (index >= 0 && index < objects.size())
        return objects[index];
    return NULL;
}

vector<Object *> Grid::getCell(int ix, int iy, int iz)
{
    return cells[ix + Nx * iy + Nx * Ny * iz];
}

int Grid::getNumObjects()
{
    return N;
}

void Grid::Build(void)
{
    //SET UP MAILBOX

    //Give objects a tag for easy access to mail box
    if (MAILBOX_ACTIVE)
    {
        for (int i = 0; i < N; i++)
        {
            Object *o = getObject(i);
            o->ID = i;
        }
        mailbox = Mailbox(N);
    }

    //SET UP ACELARATION GRID

    if (N > 0)
    {
        Object *first_obj = getObject(0);
        p0 = first_obj->GetBoundingBox().min;
        p1 = first_obj->GetBoundingBox().max;
    }

    for (int i = 0; i < N; i++) // iterate all objects
    {
        Vector min = getObject(i)->GetBoundingBox().min;
        Vector max = getObject(i)->GetBoundingBox().max;

        if (min.x < p0.x)
            p0.x = min.x;
        if (min.y < p0.y)
            p0.y = min.y;
        if (min.z < p0.z)
            p0.z = min.z;

        if (max.x > p1.x)
            p1.x = max.x;
        if (max.y > p1.y)
            p1.y = max.y;
        if (max.z > p1.z)
            p1.z = max.z;
    }

    p0.x = p0.x - EPSILON;
    p0.y = p0.y - EPSILON;
    p0.z = p0.z - EPSILON;

    p1.x = p1.x + EPSILON;
    p1.y = p1.y + EPSILON;
    p1.z = p1.z + EPSILON;

    bbox = AABB(p0, p1);

    float wx = p1.x - p0.x;
    float wy = p1.y - p0.y;
    float wz = p1.z - p0.z;

    float volume = wx * wy * wz;
    float s = pow(N / volume, (1.0f / 3.0f));

    Nx = trunc(m * wx * s) + 1;
    Ny = trunc(m * wy * s) + 1;
    Nz = trunc(m * wz * s) + 1;

    int num_cells = Nx * Ny * Nz;

    for (int c = 0; c < num_cells; c++)
    {
        cells.push_back(vector<Object *>(NULL));
    }

    for (int i = 0; i < N; i++) // iterate all objects
    {
        Object *obj = getObject(i);
        Vector min = obj->GetBoundingBox().min;
        Vector max = obj->GetBoundingBox().max;
        /* Compute indices of both cells that contain min and max coord of obj bbox */
        int ixmin = clamp((min.x - p0.x) * Nx / (p1.x - p0.x), 0, Nx - 1);
        int iymin = clamp((min.y - p0.y) * Ny / (p1.y - p0.y), 0, Ny - 1);
        int izmin = clamp((min.z - p0.z) * Nz / (p1.z - p0.z), 0, Nz - 1);

        int ixmax = clamp((max.x - p0.x) * Nx / (p1.x - p0.x), 0, Nx - 1);
        int iymax = clamp((max.y - p0.y) * Ny / (p1.y - p0.y), 0, Ny - 1);
        int izmax = clamp((max.z - p0.z) * Nz / (p1.z - p0.z), 0, Nz - 1);

        /* insert obj to the overlaped cells */
        for (int iz = izmin; iz <= izmax; iz++)
        {
            for (int iy = iymin; iy <= iymax; iy++)
            {
                for (int ix = ixmin; ix <= ixmax; ix++)
                {
                    int index = ix + Nx * iy + Nx * Ny * iz;
                    cells[index].push_back(obj);
                }
            }
        }
    }
}

bool cellClosestHit(std::vector<Object *> cellObjs, Ray &ray, float &closets_dist, Object *&closest_obj, int r_id, Mailbox mailbox)
{
    closets_dist = std::numeric_limits<float>::max();
    closest_obj = NULL;

    for (auto obj : cellObjs)
    {
        float dist;

        if (MAILBOX_ACTIVE)
        {
            bool hit;
            if (!mailbox.check(obj->ID, r_id, dist, hit)) //MISS in the mailbox
            {
                // must calculate intercec
                bool ret_interc = obj->intercepts(ray, dist);
                // and update mailbox
                mailbox.update(obj->ID, r_id, dist, ret_interc);

                if (ret_interc && dist < closets_dist)
                {
                    closest_obj = obj;
                    closets_dist = dist;
                }
            }
            else //HIT in the mailbox
            {
                if (hit && dist < closets_dist) // no need to calculate interc
                {
                    closest_obj = obj;
                    closets_dist = dist;
                }
            }
        }
        else
        {
            if (obj->intercepts(ray, dist) && dist < closets_dist)
            {
                closest_obj = obj;
                closets_dist = dist;
            }
        }
    }
    if (closest_obj == NULL)
        return false;
    return true;
}

void Grid::ray_Traverse(Ray &ray, float &closest_dist, Object *&closest_obj, int r_id)
{
    float ox = ray.origin.x;
    float oy = ray.origin.y;
    float oz = ray.origin.z;

    float dx = ray.direction.x;
    float dy = ray.direction.y;
    float dz = ray.direction.z;

    float x0 = p0.x;
    float y0 = p0.y;
    float z0 = p0.z;
    float x1 = p1.x;
    float y1 = p1.y;
    float z1 = p1.z;

    float txMin, tyMin, tzMin;
    float txMax, tyMax, tzMax;

    float a = 1.0 / dx;
    if (a >= 0)
    {
        txMin = (x0 - ox) * a;
        txMax = (x1 - ox) * a;
    }
    else
    {
        txMin = (x1 - ox) * a;
        txMax = (x0 - ox) * a;
    }

    float b = 1.0 / dy;
    if (b >= 0)
    {
        tyMin = (y0 - oy) * b;
        tyMax = (y1 - oy) * b;
    }
    else
    {
        tyMin = (y1 - oy) * b;
        tyMax = (y0 - oy) * b;
    }

    float c = 1.0 / dz;
    if (c >= 0)
    {
        tzMin = (z0 - oz) * c;
        tzMax = (z1 - oz) * c;
    }
    else
    {
        tzMin = (z1 - oz) * c;
        tzMax = (z0 - oz) * c;
    }

    //t0 is the biggest of the tmins
    float t0, t1;

    if (txMin > tyMin)
        t0 = txMin;
    else
        t0 = tyMin;

    if (tzMin > t0)
        t0 = tzMin;

    //t1 is the smallest of the tmaxs
    if (txMax < tyMax)
        t1 = txMax;
    else
        t1 = tyMax;

    if (tzMax < t1)
        t1 = tzMax;

    if (t0 > t1 || t1 < 0)
        return;

    /* float t;
    if (t0 > 0) t = t0;
    else t = t1; */

    int ix, iy, iz;

    // FIND INITIAL CELL
    if (bbox.isInside(ray.origin))
    {
        ix = clamp((ox - x0) * Nx / (x1 - x0), 0, Nx - 1);
        iy = clamp((oy - y0) * Ny / (y1 - y0), 0, Ny - 1);
        iz = clamp((oz - z0) * Nz / (z1 - z0), 0, Nz - 1);
    }
    else
    {

        Vector p = ray.origin + ray.direction * t0;
        ix = clamp((p.x - x0) * Nx / (x1 - x0), 0, Nx - 1);
        iy = clamp((p.y - y0) * Ny / (y1 - y0), 0, Ny - 1);
        iz = clamp((p.z - z0) * Nz / (z1 - z0), 0, Nz - 1);
    }

    float dtx = (txMax - txMin) / Nx;
    float dty = (tyMax - tyMin) / Ny;
    float dtz = (tzMax - tzMin) / Nz;

    float txNext, tyNext, tzNext;
    int ixStep, iyStep, izStep;
    int ixStop, iyStop, izStop;

    if (dx > 0)
    {
        txNext = txMin + (ix + 1) * dtx;
        ixStep = +1;
        ixStop = Nx;
    }
    else
    {
        txNext = txMin + (Nx - ix) * dtx;
        ixStep = -1;
        ixStop = -1;
    }

    if (dx == 0.0)
    {
        txNext = FLT_MAX;
        ixStep = -1;
        ixStop = -1;
    }

    if (dy > 0)
    {
        tyNext = tyMin + (iy + 1) * dty;
        iyStep = +1;
        iyStop = Ny;
    }
    else
    {
        tyNext = tyMin + (Ny - iy) * dty;
        iyStep = -1;
        iyStop = -1;
    }

    if (dy == 0.0)
    {
        tyNext = FLT_MAX;
        iyStep = -1;
        iyStop = -1;
    }

    if (dz > 0)
    {
        tzNext = tzMin + (iz + 1) * dtz;
        izStep = +1;
        izStop = Nz;
    }
    else
    {
        tzNext = tzMin + (Nz - iz) * dtz;
        izStep = -1;
        izStop = -1;
    }

    if (dz == 0.0)
    {
        tzNext = FLT_MAX;
        izStep = -1;
        izStop = -1;
    }

    while (true)
    {
        int index = ix + Nx * iy + Nx * Ny * iz;
        float dist;
        Object *o;

        if (txNext < tyNext && txNext < tzNext)
        {
            if (cellClosestHit(cells[index], ray, dist, o, r_id, mailbox) && dist < txNext)
            {
                closest_obj = o;
                closest_dist = dist;
                return;
            }

            txNext += dtx;
            ix += ixStep;

            if (ix == ixStop)
                return;
        }
        else
        {
            if (tyNext < tzNext)
            {
                if (cellClosestHit(cells[index], ray, dist, o, r_id, mailbox) && dist < tyNext)
                {
                    closest_obj = o;
                    closest_dist = dist;
                    return;
                }

                tyNext += dty;
                iy += iyStep;

                if (iy == iyStop)
                    return;
            }
            else
            {
                if (cellClosestHit(cells[index], ray, dist, o, r_id, mailbox) && dist < tzNext)
                {
                    closest_obj = o;
                    closest_dist = dist;
                    return;
                }

                tzNext += dtz;
                iz += izStep;

                if (iz == izStop)
                    return;
            }
        }
    }
    return;
}

//RETURNS TRUE IF RAY INTERCEPTS ANY OF THE OBJECTS
bool safeIsObscured(std::vector<Object *> cellObjs, Ray &ray)
{
    float dist;
    for (auto obj : cellObjs)
    {
        if (obj->intercepts(ray, dist))
            return true;
    }

    return false;
}

//RETURNS TRUE IF RAY INTERCEPTS ANY OF THE OBJECTS AND SAVES ISTANCE
bool IsObscured(std::vector<Object *> cellObjs, Ray &ray, float l_dist)
{
    float dist;
    for (auto obj : cellObjs)
    {
        if (obj->intercepts(ray, dist) && dist < l_dist)
            return true;
    }

    return false;
}

// traverses grid from hit point in direction of light source, cell stop is given by l_pos
// return true if in shadow, returns false if exposed
bool Grid::shadow_Traverse(Ray &ray, float l_dist, int r_id)
{
    float ox = ray.origin.x;
    float oy = ray.origin.y;
    float oz = ray.origin.z;

    float dx = ray.direction.x;
    float dy = ray.direction.y;
    float dz = ray.direction.z;

    float x0 = p0.x;
    float y0 = p0.y;
    float z0 = p0.z;
    float x1 = p1.x;
    float y1 = p1.y;
    float z1 = p1.z;

    float txMin, tyMin, tzMin;
    float txMax, tyMax, tzMax;

    float a = 1.0 / dx;
    if (a >= 0)
    {
        txMin = (x0 - ox) * a;
        txMax = (x1 - ox) * a;
    }
    else
    {
        txMin = (x1 - ox) * a;
        txMax = (x0 - ox) * a;
    }

    float b = 1.0 / dy;
    if (b >= 0)
    {
        tyMin = (y0 - oy) * b;
        tyMax = (y1 - oy) * b;
    }
    else
    {
        tyMin = (y1 - oy) * b;
        tyMax = (y0 - oy) * b;
    }

    float c = 1.0 / dz;
    if (c >= 0)
    {
        tzMin = (z0 - oz) * c;
        tzMax = (z1 - oz) * c;
    }
    else
    {
        tzMin = (z1 - oz) * c;
        tzMax = (z0 - oz) * c;
    }

    float t0, t1;

    if (txMin > tyMin)
        t0 = txMin;
    else
        t0 = tyMin;

    if (tzMin > t0)
        t0 = tzMin;

    if (txMax < tyMax)
        t1 = txMax;
    else
        t1 = tyMax;

    if (tzMax < t1)
        t1 = tzMax;

    if (t0 > t1)
        return false;

    int ix, iy, iz;

    // FIND INITIAL CELL
    if (bbox.isInside(ray.origin))
    {
        ix = clamp((ox - x0) * Nx / (x1 - x0), 0, Nx - 1);
        iy = clamp((oy - y0) * Ny / (y1 - y0), 0, Ny - 1);
        iz = clamp((oz - z0) * Nz / (z1 - z0), 0, Nz - 1);
    }
    else
    {

        Vector p = ray.origin + ray.direction * t0;
        ix = clamp((p.x - x0) * Nx / (x1 - x0), 0, Nx - 1);
        iy = clamp((p.y - y0) * Ny / (y1 - y0), 0, Ny - 1);
        iz = clamp((p.z - z0) * Nz / (z1 - z0), 0, Nz - 1);
    }

    float dtx = (txMax - txMin) / Nx;
    float dty = (tyMax - tyMin) / Ny;
    float dtz = (tzMax - tzMin) / Nz;

    float txNext, tyNext, tzNext;
    int ixStep, iyStep, izStep;
    int ixStop, iyStop, izStop;

    if (dx > 0)
    {
        txNext = txMin + (ix + 1) * dtx;
        ixStep = +1;
        ixStop = Nx;
    }
    else
    {
        txNext = txMin + (Nx - ix) * dtx;
        ixStep = -1;
        ixStop = -1;
    }

    if (dx == 0.0)
    {
        txNext = FLT_MAX;
        ixStep = -1;
        ixStop = -1;
    }

    if (dy > 0)
    {
        tyNext = tyMin + (iy + 1) * dty;
        iyStep = +1;
        iyStop = Ny;
    }
    else
    {
        tyNext = tyMin + (Ny - iy) * dty;
        iyStep = -1;
        iyStop = -1;
    }

    if (dy == 0.0)
    {
        tyNext = FLT_MAX;
        iyStep = -1;
        iyStop = -1;
    }

    if (dz > 0)
    {
        tzNext = tzMin + (iz + 1) * dtz;
        izStep = +1;
        izStop = Nz;
    }
    else
    {
        tzNext = tzMin + (Nz - iz) * dtz;
        izStep = -1;
        izStop = -1;
    }

    if (dz == 0.0)
    {
        tzNext = FLT_MAX;
        izStep = -1;
        izStop = -1;
    }

    while (true)
    {
        int index = ix + Nx * iy + Nx * Ny * iz;
        float dist;
        Object *o;

        if (txNext < tyNext && txNext < tzNext)
        {
            if (IsObscured(cells[index], ray, l_dist))
            {
                return true;
            }

            txNext += dtx;
            ix += ixStep;

            if (ix == ixStop)
                return false;
        }
        else
        {
            if (tyNext < tzNext)
            {
                if (IsObscured(cells[index], ray, l_dist))
                {
                    return true;
                }

                tyNext += dty;
                iy += iyStep;

                if (iy == iyStop)
                    return false;
            }
            else
            {
                if (IsObscured(cells[index], ray, l_dist))
                {
                    return true;
                }

                tzNext += dtz;
                iz += izStep;

                if (iz == izStop)
                    return false;
            }
        }
    }

    return false;
}

// traverses grid from hit point in direction of light source, cell stop is given by l_pos
// return true if in shadow, returns false if exposed
bool Grid::safe_shadow_Traverse(Ray &ray, int r_id)
{

    float ox = ray.origin.x;
    float oy = ray.origin.y;
    float oz = ray.origin.z;

    float dx = ray.direction.x;
    float dy = ray.direction.y;
    float dz = ray.direction.z;

    float x0 = p0.x;
    float y0 = p0.y;
    float z0 = p0.z;
    float x1 = p1.x;
    float y1 = p1.y;
    float z1 = p1.z;

    float txMin, tyMin, tzMin;
    float txMax, tyMax, tzMax;

    float a = 1.0 / dx;
    if (a >= 0)
    {
        txMin = (x0 - ox) * a;
        txMax = (x1 - ox) * a;
    }
    else
    {
        txMin = (x1 - ox) * a;
        txMax = (x0 - ox) * a;
    }

    float b = 1.0 / dy;
    if (b >= 0)
    {
        tyMin = (y0 - oy) * b;
        tyMax = (y1 - oy) * b;
    }
    else
    {
        tyMin = (y1 - oy) * b;
        tyMax = (y0 - oy) * b;
    }

    float c = 1.0 / dz;
    if (c >= 0)
    {
        tzMin = (z0 - oz) * c;
        tzMax = (z1 - oz) * c;
    }
    else
    {
        tzMin = (z1 - oz) * c;
        tzMax = (z0 - oz) * c;
    }

    float t0, t1;

    if (txMin > tyMin)
        t0 = txMin;
    else
        t0 = tyMin;

    if (tzMin > t0)
        t0 = tzMin;

    if (txMax < tyMax)
        t1 = txMax;
    else
        t1 = tyMax;

    if (tzMax < t1)
        t1 = tzMax;

    if (t0 > t1)
        return false;

    int ix, iy, iz;

    // FIND INITIAL CELL
    if (bbox.isInside(ray.origin))
    {
        ix = clamp((ox - x0) * Nx / (x1 - x0), 0, Nx - 1);
        iy = clamp((oy - y0) * Ny / (y1 - y0), 0, Ny - 1);
        iz = clamp((oz - z0) * Nz / (z1 - z0), 0, Nz - 1);
    }
    else
    {

        Vector p = ray.origin + ray.direction * t0;
        ix = clamp((p.x - x0) * Nx / (x1 - x0), 0, Nx - 1);
        iy = clamp((p.y - y0) * Ny / (y1 - y0), 0, Ny - 1);
        iz = clamp((p.z - z0) * Nz / (z1 - z0), 0, Nz - 1);
    }

    float dtx = (txMax - txMin) / Nx;
    float dty = (tyMax - tyMin) / Ny;
    float dtz = (tzMax - tzMin) / Nz;

    float txNext, tyNext, tzNext;
    int ixStep, iyStep, izStep;
    int ixStop, iyStop, izStop;

    if (dx > 0)
    {
        txNext = txMin + (ix + 1) * dtx;
        ixStep = +1;
        ixStop = Nx;
    }
    else
    {
        txNext = txMin + (Nx - ix) * dtx;
        ixStep = -1;
        ixStop = -1;
    }

    if (dx == 0.0)
    {
        txNext = FLT_MAX;
        ixStep = -1;
        ixStop = -1;
    }

    if (dy > 0)
    {
        tyNext = tyMin + (iy + 1) * dty;
        iyStep = +1;
        iyStop = Ny;
    }
    else
    {
        tyNext = tyMin + (Ny - iy) * dty;
        iyStep = -1;
        iyStop = -1;
    }

    if (dy == 0.0)
    {
        tyNext = FLT_MAX;
        iyStep = -1;
        iyStop = -1;
    }

    if (dz > 0)
    {
        tzNext = tzMin + (iz + 1) * dtz;
        izStep = +1;
        izStop = Nz;
    }
    else
    {
        tzNext = tzMin + (Nz - iz) * dtz;
        izStep = -1;
        izStop = -1;
    }

    if (dz == 0.0)
    {
        tzNext = FLT_MAX;
        izStep = -1;
        izStop = -1;
    }

    while (true)
    {
        int index = ix + Nx * iy + Nx * Ny * iz;
        float dist;
        Object *o;

        if (txNext < tyNext && txNext < tzNext)
        {
            if (safeIsObscured(cells[index], ray))
            {
                return true;
            }

            txNext += dtx;
            ix += ixStep;

            if (ix == ixStop)
                return false;
        }
        else
        {
            if (tyNext < tzNext)
            {
                if (safeIsObscured(cells[index], ray))
                {
                    return true;
                }

                tyNext += dty;
                iy += iyStep;

                if (iy == iyStop)
                    return false;
            }
            else
            {
                if (safeIsObscured(cells[index], ray))
                {
                    return true;
                }

                tzNext += dtz;
                iz += izStep;

                if (iz == izStop)
                    return false;
            }
        }
    }

    return false;
}